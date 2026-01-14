#!/usr/bin/env python3
# sony_fms_mock_server.py
#
# Sony FleetManager (FMS) の代替として動く「最小 gRPC モックサーバ」です。
# fleet_adapter_sony が呼び出す代表的なRPCを成功応答し、FleetState を stream 配信します。
#
# 重要:
# - fleet_adapter_sony 側のソースは変更不要（config.yaml の sony: 設定だけでOK）
# - proto 生成済み（*_pb2.py / *_pb2_grpc.py）が Python から import できる必要があります
#
from __future__ import annotations

import argparse
import hashlib
import importlib
import threading
import time
from concurrent import futures
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

import grpc
import yaml


# =========================
# Logging interceptor
# =========================
class _LoggingInterceptor(grpc.ServerInterceptor):
    """呼び出されたRPC名を標準出力に出す（疎通確認用）"""

    def intercept_service(self, continuation, handler_call_details):
        method = handler_call_details.method
        print(f"[sony-fms-mock][rpc] {method}")
        return continuation(handler_call_details)


# =========================
# Utilities: proto lookup
# =========================
def _iter_pb2_modules_from_pb2_grpc(pb2_grpc_mod: Any) -> List[Any]:
    """Generated *_pb2_grpc.py typically imports pb2 as something like job__pb2."""
    mods: List[Any] = []
    for name, obj in getattr(pb2_grpc_mod, "__dict__", {}).items():
        if name.endswith("__pb2") and hasattr(obj, "DESCRIPTOR"):
            mods.append(obj)
    return mods


def _find_symbol(mods: Iterable[Any], name: str) -> Optional[Any]:
    for m in mods:
        if hasattr(m, name):
            return getattr(m, name)
    return None


def _new_message(mods: Iterable[Any], name: str) -> Any:
    cls = _find_symbol(mods, name)
    if cls is None:
        raise RuntimeError(f"protobuf message '{name}' not found in given modules")
    return cls()


def _set_success_header(all_pb2_mods: List[Any], resp: Any) -> None:
    """
    Best-effort to set:
      resp.header.status.code = STATUS_CODE_SUCCESS
    If not present, silently ignore (mock should stay resilient).
    """
    header = getattr(resp, "header", None)
    if header is None:
        return
    status = getattr(header, "status", None)
    if status is None:
        return

    # Try to resolve enum value
    enum_mod = _find_symbol(all_pb2_mods, "StatusCode")
    if enum_mod is not None and hasattr(enum_mod, "STATUS_CODE_SUCCESS"):
        success_val = int(getattr(enum_mod, "STATUS_CODE_SUCCESS"))
    else:
        # Commonly SUCCESS is 1
        success_val = 1

    try:
        status.code = success_val
    except Exception:
        pass

    try:
        status.message = ""
    except Exception:
        pass


# =========================
# Mock state
# =========================
@dataclass
class AgentRuntime:
    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0
    battery_pct: float = 100.0
    paused: bool = False


@dataclass
class MoveJobRuntime:
    job_id: str
    agent_id: str
    required_tag: str
    angle_rad: Optional[float] = None
    status: int = 4  # RUNNING-ish (mock)
    # Sony backend considers completed/canceled/failed as: 5/6/7
    # We'll move RUNNING(4) -> COMPLETED(5) after a delay
    start_time: float = field(default_factory=time.time)
    complete_after_sec: float = 2.0


class SonyFmsMockCore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._agents: Dict[str, AgentRuntime] = {}
        self._move_jobs: Dict[str, MoveJobRuntime] = {}

        self._stop_evt = threading.Event()
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._tick_thread.start()

    def shutdown(self) -> None:
        self._stop_evt.set()
        self._tick_thread.join(timeout=1.0)

    def ensure_agent(self, agent_id: str) -> AgentRuntime:
        with self._lock:
            if agent_id not in self._agents:
                self._agents[agent_id] = AgentRuntime()
            return self._agents[agent_id]

    def _tag_to_xy(self, required_tag: str) -> Tuple[float, float]:
        # Stable pseudo-coordinate generator (so same tag -> same x,y)
        h = hashlib.sha256(required_tag.encode("utf-8")).digest()
        a = int.from_bytes(h[0:4], "little", signed=False) / 0xFFFFFFFF
        b = int.from_bytes(h[4:8], "little", signed=False) / 0xFFFFFFFF
        # Keep within a reasonable range for demo maps
        return (a * 10.0, b * 10.0)

    def register_move_jobs(self, move_jobs: Iterable[Any]) -> None:
        with self._lock:
            for mj in move_jobs:
                job_id = str(getattr(mj, "job_id", "")).strip()
                if not job_id:
                    continue

                # agent.targets.ids[0]
                agent_req = getattr(mj, "agent", None)
                targets = getattr(agent_req, "targets", None) if agent_req is not None else None
                ids = getattr(targets, "ids", None) if targets is not None else None
                agent_id = str(ids[0]) if ids else "unknown_agent"
                self.ensure_agent(agent_id)

                # position.candidates[0].required_tags[0]
                pos_req = getattr(mj, "position", None)
                cands = getattr(pos_req, "candidates", None) if pos_req is not None else None
                cand0 = cands[0] if cands else None
                rtags = getattr(cand0, "required_tags", None) if cand0 is not None else None
                required_tag = str(rtags[0]) if rtags else "1/unknown"

                angle = getattr(cand0, "angle", None) if cand0 is not None else None
                angle_rad = float(angle) if angle is not None else None

                self._move_jobs[job_id] = MoveJobRuntime(
                    job_id=job_id,
                    agent_id=agent_id,
                    required_tag=required_tag,
                    angle_rad=angle_rad,
                    status=4,
                    start_time=time.time(),
                    complete_after_sec=2.0,
                )

    def start_jobs(self, job_ids: Iterable[str]) -> None:
        with self._lock:
            now = time.time()
            for jid in job_ids:
                job = self._move_jobs.get(str(jid))
                if job is None:
                    continue
                job.status = 4
                job.start_time = now

    def cancel_jobs(self, job_ids: Iterable[str]) -> None:
        with self._lock:
            for jid in job_ids:
                job = self._move_jobs.get(str(jid))
                if job is None:
                    continue
                job.status = 6  # CANCELED

    def pause_agent(self, agent_ids: Iterable[str]) -> None:
        with self._lock:
            for aid in agent_ids:
                a = self.ensure_agent(str(aid))
                a.paused = True
            # Mark active jobs for those agents as "PAUSED" (8) for realism
            for job in self._move_jobs.values():
                if job.agent_id in set(map(str, agent_ids)) and job.status == 4:
                    job.status = 8

    def resume_agent(self, agent_ids: Iterable[str]) -> None:
        with self._lock:
            for aid in agent_ids:
                a = self.ensure_agent(str(aid))
                a.paused = False
            # Resume paused jobs back to running
            for job in self._move_jobs.values():
                if job.agent_id in set(map(str, agent_ids)) and job.status == 8:
                    job.status = 4
                    job.start_time = time.time()

    def job_states(self, job_ids: Optional[Iterable[str]]) -> List[MoveJobRuntime]:
        with self._lock:
            if not job_ids:
                return list(self._move_jobs.values())
            s = set(map(str, job_ids))
            return [j for j in self._move_jobs.values() if j.job_id in s]

    def agent_snapshot(self) -> Dict[str, AgentRuntime]:
        with self._lock:
            return {k: v for k, v in self._agents.items()}

    def _tick_loop(self) -> None:
        while not self._stop_evt.is_set():
            time.sleep(0.1)
            now = time.time()
            with self._lock:
                # Battery drain
                for a in self._agents.values():
                    if not a.paused:
                        a.battery_pct = max(0.0, a.battery_pct - 0.01)

                # Progress jobs
                for job in self._move_jobs.values():
                    if job.status not in (4,):  # running only
                        continue
                    agent = self._agents.get(job.agent_id)
                    if agent is None or agent.paused:
                        continue

                    # Move towards target (simple interpolation)
                    tx, ty = self._tag_to_xy(job.required_tag)
                    agent.x += (tx - agent.x) * 0.1
                    agent.y += (ty - agent.y) * 0.1
                    if job.angle_rad is not None:
                        agent.yaw_rad = job.angle_rad

                    # Complete after delay
                    if (now - job.start_time) >= job.complete_after_sec:
                        job.status = 5  # COMPLETED
                        agent.x, agent.y = tx, ty


# =========================
# Servicer factory
# =========================
class ServicerFactory:
    def __init__(self, core: SonyFmsMockCore, all_pb2_mods: List[Any]) -> None:
        self._core = core
        self._all_pb2_mods = all_pb2_mods

    def create_servicer(self, pb2_grpc_mod: Any, base_servicer_cls: Any) -> Any:
        pb2_mods_for_service = _iter_pb2_modules_from_pb2_grpc(pb2_grpc_mod)
        # Fallback to global pool for shared messages
        msg_pool = list(pb2_mods_for_service) + list(self._all_pb2_mods)

        def _mk_unary_success(method_name: str) -> Any:
            resp_name = f"{method_name}Response"
            resp = _new_message(msg_pool, resp_name)
            _set_success_header(self._all_pb2_mods, resp)
            return resp

        def _extract_job_ids_from_filter(req: Any) -> List[str]:
            jf = getattr(req, "job_filter", None)
            if jf is None:
                return []
            jids = getattr(jf, "job_ids", None)
            if not jids:
                return []
            return [str(x) for x in jids]

        def _extract_agent_ids_from_requirement(req: Any) -> List[str]:
            agent_req = getattr(req, "agent", None)
            targets = getattr(agent_req, "targets", None) if agent_req is not None else None
            ids = getattr(targets, "ids", None) if targets is not None else None
            if not ids:
                return []
            return [str(x) for x in ids]

        # ---- Implementations ----
        def MonitorFleetState(self, request: Any, context: Any):
            resp_cls = _find_symbol(msg_pool, "MonitorFleetStateResponse")
            if resp_cls is None:
                resp_cls = _find_symbol(msg_pool, "GetStateResponse")
            if resp_cls is None:
                context.abort(grpc.StatusCode.UNIMPLEMENTED, "MonitorFleetStateResponse not found")

            interval_sec = 1.0
            interval = getattr(request, "interval", None)
            if interval is not None and hasattr(interval, "seconds"):
                try:
                    sec = float(interval.seconds)
                    nanos = float(getattr(interval, "nanos", 0))
                    interval_sec = max(0.1, sec + nanos / 1e9)
                except Exception:
                    pass

            while context.is_active():
                resp = resp_cls()
                _set_success_header(self._all_pb2_mods, resp)

                agents_field = getattr(resp, "agents", None)
                if agents_field is not None:
                    snap = self._core.agent_snapshot()
                    agent_state_cls = _find_symbol(msg_pool, "AgentState")
                    for agent_id, a in snap.items():
                        if agent_state_cls is None:
                            break
                        st = agent_state_cls()

                        agent_info = getattr(st, "agent_info", None)
                        if agent_info is not None and hasattr(agent_info, "id"):
                            agent_info.id = agent_id

                        nav_state = getattr(st, "navigation_state", None)
                        pose = getattr(nav_state, "current_pose", None) if nav_state is not None else None
                        if pose is not None:
                            pos = getattr(pose, "position", None)
                            if pos is not None:
                                try:
                                    pos.x = float(a.x)
                                    pos.y = float(a.y)
                                    pos.z = 0.0
                                except Exception:
                                    pass
                            q = getattr(pose, "orientation", None)
                            if q is not None:
                                try:
                                    import math
                                    half = a.yaw_rad * 0.5
                                    q.x, q.y = 0.0, 0.0
                                    q.z = math.sin(half)
                                    q.w = math.cos(half)
                                except Exception:
                                    pass

                        sys_state = getattr(st, "system_state", None)
                        bat = getattr(sys_state, "battery", None) if sys_state is not None else None
                        if bat is not None and hasattr(bat, "percentage"):
                            try:
                                bat.percentage = float(a.battery_pct)
                            except Exception:
                                pass

                        agents_field.append(st)

                yield resp
                time.sleep(interval_sec)

        def GetState(self, request: Any, context: Any):
            # Job.GetState か Fleet.GetState の両対応（リクエスト構造で判定）
            if hasattr(request, "job_filter"):
                resp = _new_message(msg_pool, "GetStateResponse")
                _set_success_header(self._all_pb2_mods, resp)

                state = getattr(resp, "state", None)
                move_job_state_field = getattr(state, "move_job_state", None) if state is not None else None
                if move_job_state_field is None:
                    return resp

                job_ids = _extract_job_ids_from_filter(request)
                jobs = self._core.job_states(job_ids if job_ids else None)

                move_state_cls = _find_symbol(msg_pool, "MoveJobState")
                if move_state_cls is None:
                    return resp

                for j in jobs:
                    ms = move_state_cls()
                    if hasattr(ms, "job_id"):
                        ms.job_id = j.job_id
                    if hasattr(ms, "status"):
                        ms.status = int(j.status)
                    move_job_state_field.append(ms)
                return resp

            # Fleet GetState
            resp = _new_message(msg_pool, "GetStateResponse")
            _set_success_header(self._all_pb2_mods, resp)

            agents_field = getattr(resp, "agents", None)
            if agents_field is None:
                return resp

            snap = self._core.agent_snapshot()
            agent_state_cls = _find_symbol(msg_pool, "AgentState")
            if agent_state_cls is None:
                return resp

            for agent_id, a in snap.items():
                st = agent_state_cls()
                agent_info = getattr(st, "agent_info", None)
                if agent_info is not None and hasattr(agent_info, "id"):
                    agent_info.id = agent_id

                nav_state = getattr(st, "navigation_state", None)
                pose = getattr(nav_state, "current_pose", None) if nav_state is not None else None
                if pose is not None:
                    pos = getattr(pose, "position", None)
                    if pos is not None:
                        try:
                            pos.x = float(a.x)
                            pos.y = float(a.y)
                            pos.z = 0.0
                        except Exception:
                            pass

                sys_state = getattr(st, "system_state", None)
                bat = getattr(sys_state, "battery", None) if sys_state is not None else None
                if bat is not None and hasattr(bat, "percentage"):
                    try:
                        bat.percentage = float(a.battery_pct)
                    except Exception:
                        pass

                agents_field.append(st)

            return resp

        def RegisterJobs(self, request: Any, context: Any):
            move_jobs = getattr(request, "move_jobs", None)
            if move_jobs:
                self._core.register_move_jobs(move_jobs)
            return _mk_unary_success("RegisterJobs")

        def StartJobs(self, request: Any, context: Any):
            job_ids = _extract_job_ids_from_filter(request)
            self._core.start_jobs(job_ids)
            return _mk_unary_success("StartJobs")

        def CancelJobs(self, request: Any, context: Any):
            job_ids = getattr(request, "job_ids", None) or []
            self._core.cancel_jobs([str(x) for x in job_ids])
            return _mk_unary_success("CancelJobs")

        def PauseAgent(self, request: Any, context: Any):
            agent_ids = _extract_agent_ids_from_requirement(request)
            self._core.pause_agent(agent_ids)
            return _mk_unary_success("PauseAgent")

        def ResumeAgent(self, request: Any, context: Any):
            agent_ids = _extract_agent_ids_from_requirement(request)
            self._core.resume_agent(agent_ids)
            return _mk_unary_success("ResumeAgent")

        # Generic unary fallback
        def _fallback_unary(self, request: Any, context: Any, method_name: str):
            return _mk_unary_success(method_name)

        methods: Dict[str, Any] = {}

        for name in dir(base_servicer_cls):
            if name.startswith("_"):
                continue
            attr = getattr(base_servicer_cls, name, None)
            if not callable(attr):
                continue

            if name == "MonitorFleetState":
                methods[name] = MonitorFleetState
            elif name == "GetState":
                methods[name] = GetState
            elif name == "RegisterJobs":
                methods[name] = RegisterJobs
            elif name == "StartJobs":
                methods[name] = StartJobs
            elif name == "CancelJobs":
                methods[name] = CancelJobs
            elif name == "PauseAgent":
                methods[name] = PauseAgent
            elif name == "ResumeAgent":
                methods[name] = ResumeAgent
            else:
                methods[name] = (lambda mn: (lambda self, req, ctx: _fallback_unary(self, req, ctx, mn)))(name)

        Impl = type(f"Mock_{base_servicer_cls.__name__}", (base_servicer_cls,), methods)
        return Impl()


# =========================
# Main
# =========================
def _load_config(path: str) -> Mapping[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="fleet_adapter_sony/config.yaml のパス")
    args = ap.parse_args()

    cfg = _load_config(args.config)
    sony = cfg.get("sony")
    if not isinstance(sony, Mapping):
        raise SystemExit("config.yaml に sony: セクションがありません（コメントアウトを解除してください）")

    grpc_cfg = sony.get("grpc", {}) or {}
    target = str(grpc_cfg.get("target", "")).strip()
    if not target:
        raise SystemExit("sony.grpc.target が未設定です（例: 127.0.0.1:50051）")

    if ":" not in target:
        raise SystemExit("sony.grpc.target は host:port 形式にしてください（例: 127.0.0.1:50051）")
    _, port_str = target.rsplit(":", 1)
    bind_addr = f"0.0.0.0:{port_str}"

    proto_modules = sony.get("proto_modules", []) or []
    if not isinstance(proto_modules, list) or not proto_modules:
        raise SystemExit("sony.proto_modules が未設定です（pb2/pb2_grpc の module 名を列挙してください）")

    pb2_mods: List[Any] = []
    pb2_grpc_mods: List[Any] = []
    for mod_name in proto_modules:
        m = importlib.import_module(str(mod_name))
        if any(k.endswith("Stub") for k in getattr(m, "__dict__", {}).keys()):
            pb2_grpc_mods.append(m)
        else:
            pb2_mods.append(m)

    if not pb2_grpc_mods:
        raise SystemExit("proto_modules に *_pb2_grpc が含まれていません")

    core = SonyFmsMockCore()
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=10),
        interceptors=[_LoggingInterceptor()],
    )
    factory = ServicerFactory(core=core, all_pb2_mods=pb2_mods)

    registered = 0
    for m in pb2_grpc_mods:
        add_fns = [
            (n, o)
            for n, o in getattr(m, "__dict__", {}).items()
            if n.startswith("add_") and n.endswith("_to_server") and callable(o)
        ]
        for add_name, add_fn in add_fns:
            base_name = add_name[len("add_") : -len("_to_server")]
            base_cls = getattr(m, base_name, None)
            if base_cls is None:
                continue
            servicer = factory.create_servicer(m, base_cls)
            add_fn(servicer, server)
            registered += 1

    if registered == 0:
        raise SystemExit("サービス登録に失敗しました（pb2_grpc の add_*_to_server が見つかりません）")

    server.add_insecure_port(bind_addr)
    server.start()
    print(f"[sony-fms-mock] listening on {bind_addr} (target={target}) services={registered}")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        core.shutdown()
        server.stop(grace=None)


if __name__ == "__main__":
    main()
