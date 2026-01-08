from __future__ import annotations

import math
import threading
import time
import uuid
import importlib
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


# -----------------------------------------------------------------------------
# Template RobotClientAPI
# -----------------------------------------------------------------------------
# このファイルは「外部フリートAPIの薄いラッパ」です。
#
# 目的:
# - FleetAdapter / RobotCommandHandle から呼ばれる API 群をひとまとめにする
# - 各社差分は "このファイル（= RobotClientAPI）" と config.yaml に閉じ込める
#
# 方針（Sony を想定した拡張）:
# - 既存テンプレの public メソッドは維持（position/navigate/...）
# - "座標ベース" だけでなく "タグ/ノードIDベース" の移動指令にも対応できるよう、
#   navigate() に optional 引数 (waypoint_name, angle_rad) を追加（後方互換）
#
# NOTE:
# - gRPC の proto(pb2) は、このテンプレートには同梱しません。
#   Sony から提供される proto を python へ生成し、PYTHONPATH に載せた上で、
#   config.yaml で module 名を指定してください。
# -----------------------------------------------------------------------------


@dataclass
class _RobotSimState:
    x: float = 0.0
    y: float = 0.0
    theta_deg: float = 0.0

    # navigation
    dest_x: Optional[float] = None
    dest_y: Optional[float] = None
    dest_theta_deg: Optional[float] = None
    moving: bool = False
    paused: bool = False

    # process / docking (simple timer-based)
    process_end_time: Optional[float] = None

    # battery (0.0-1.0)
    battery: float = 1.0


class _BackendBase:
    def check_connection(self) -> bool:
        raise NotImplementedError

    def position(self, robot_name: str) -> Optional[List[float]]:
        raise NotImplementedError

    def navigate(
        self,
        robot_name: str,
        destination: List[float],
        map_name: str,
        *,
        waypoint_name: Optional[str] = None,
        angle_rad: Optional[float] = None,
        **_: Any,
    ) -> bool:
        raise NotImplementedError

    def stop(self, robot_name: str) -> bool:
        raise NotImplementedError

    def navigation_remaining_duration(self, robot_name: str) -> float:
        return 0.0

    def navigation_completed(self, robot_name: str) -> bool:
        raise NotImplementedError

    def start_process(self, robot_name: str, process: str, map_name: str) -> bool:
        raise NotImplementedError

    def process_completed(self, robot_name: str) -> bool:
        raise NotImplementedError

    def docking_completed(self, robot_name: str) -> bool:
        # 既存テンプレが docking_completed を呼ぶ実装になっているケースがあるため、alias を持たせる
        return self.process_completed(robot_name)

    def pause(self, robot_name: str) -> bool:
        raise NotImplementedError

    def resume(self, robot_name: str) -> bool:
        raise NotImplementedError

    def battery_soc(self, robot_name: str) -> Optional[float]:
        raise NotImplementedError

    def shutdown(self) -> None:
        return


# -----------------------------------------------------------------------------
# Simulation backend (default)
# -----------------------------------------------------------------------------
class _SimBackend(_BackendBase):
    """テンプレート用の「動く」RobotClientAPI（シミュレーション実装）。

    各社の外部APIが未接続でも、FleetAdapterの基本配線や状態遷移の確認ができるように、
    シンプルな運動モデルでナビゲーションを模擬します。

    - position(): 現在位置 [x, y, theta_deg]
    - navigate(): 目標に向けて移動を開始
    - navigation_completed(): 目標到達判定
    - stop()/pause()/resume()
    - start_process()/process_completed()
    - battery_soc(): 移動中に緩やかに減少（充電はしない）
    """

    def __init__(self) -> None:
        # Simulation parameters
        self._update_hz = 20.0
        self._linear_speed_mps = 0.5
        self._arrive_dist_m = 0.05

        self._lock = threading.Lock()
        self._robots: Dict[str, _RobotSimState] = {}

        self.connected = True
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # helpers
    def _ensure_robot(self, robot_name: str) -> _RobotSimState:
        with self._lock:
            if robot_name not in self._robots:
                self._robots[robot_name] = _RobotSimState()
            return self._robots[robot_name]

    def _loop(self) -> None:
        dt = 1.0 / self._update_hz
        while not self._stop_evt.is_set():
            t0 = time.time()
            with self._lock:
                for st in self._robots.values():
                    # process
                    if st.process_end_time is not None and time.time() >= st.process_end_time:
                        st.process_end_time = None

                    if not st.moving or st.paused:
                        continue

                    if st.dest_x is None or st.dest_y is None:
                        st.moving = False
                        continue

                    dx = st.dest_x - st.x
                    dy = st.dest_y - st.y
                    dist = math.hypot(dx, dy)

                    if dist <= self._arrive_dist_m:
                        # snap to destination
                        st.x = st.dest_x
                        st.y = st.dest_y
                        if st.dest_theta_deg is not None:
                            st.theta_deg = st.dest_theta_deg
                        st.moving = False
                        st.dest_x = st.dest_y = st.dest_theta_deg = None
                        continue

                    # Move towards destination
                    step = self._linear_speed_mps * dt
                    if step > dist:
                        step = dist
                    st.x += (dx / dist) * step
                    st.y += (dy / dist) * step

                    # Battery drain (simple)
                    st.battery = max(0.0, st.battery - 0.00005)

            elapsed = time.time() - t0
            sleep = dt - elapsed
            if sleep > 0:
                time.sleep(sleep)

    # API
    def check_connection(self) -> bool:
        return True

    def position(self, robot_name: str) -> Optional[List[float]]:
        st = self._ensure_robot(robot_name)
        with self._lock:
            return [st.x, st.y, st.theta_deg]

    def navigate(
        self,
        robot_name: str,
        destination: List[float],
        map_name: str,
        *,
        waypoint_name: Optional[str] = None,
        angle_rad: Optional[float] = None,
        **_: Any,
    ) -> bool:
        _ = (map_name, waypoint_name, angle_rad)
        st = self._ensure_robot(robot_name)
        try:
            x, y, theta = float(destination[0]), float(destination[1]), float(destination[2])
        except Exception:
            return False

        with self._lock:
            st.dest_x = x
            st.dest_y = y
            st.dest_theta_deg = theta
            st.moving = True
        return True

    def stop(self, robot_name: str) -> bool:
        st = self._ensure_robot(robot_name)
        with self._lock:
            st.moving = False
            st.dest_x = st.dest_y = st.dest_theta_deg = None
        return True

    def navigation_remaining_duration(self, robot_name: str) -> float:
        st = self._ensure_robot(robot_name)
        with self._lock:
            if not st.moving or st.dest_x is None or st.dest_y is None:
                return 0.0
            dist = math.hypot(st.dest_x - st.x, st.dest_y - st.y)
        return dist / max(1e-6, self._linear_speed_mps)

    def navigation_completed(self, robot_name: str) -> bool:
        st = self._ensure_robot(robot_name)
        with self._lock:
            return not st.moving

    def start_process(self, robot_name: str, process: str, map_name: str) -> bool:
        _ = (process, map_name)
        # Simulate a fixed-duration process (e.g., docking / load/unload)
        st = self._ensure_robot(robot_name)
        with self._lock:
            st.process_end_time = time.time() + 2.0
        return True

    def process_completed(self, robot_name: str) -> bool:
        st = self._ensure_robot(robot_name)
        with self._lock:
            return st.process_end_time is None

    def pause(self, robot_name: str) -> bool:
        st = self._ensure_robot(robot_name)
        with self._lock:
            st.paused = True
        return True

    def resume(self, robot_name: str) -> bool:
        st = self._ensure_robot(robot_name)
        with self._lock:
            st.paused = False
        return True

    def battery_soc(self, robot_name: str) -> Optional[float]:
        st = self._ensure_robot(robot_name)
        with self._lock:
            return float(st.battery)

    def shutdown(self) -> None:
        self._stop_evt.set()
        self._thread.join(timeout=1.0)


# -----------------------------------------------------------------------------
# Sony gRPC backend (optional)
# -----------------------------------------------------------------------------
class _SonyGrpcBackend(_BackendBase):
    """Sony FMS gRPC を使う backend。

    必要な設定は config.yaml の root に sony セクションとして渡す想定です（例）:

    sony:
      grpc:
        target: "127.0.0.1:50051"
        secure: false
        metadata:
          # 任意: 認証が必要なら追加
          # authorization: "Bearer xxx"
      proto_modules:
        # pb2 / pb2_grpc を含む module 名（生成物）
        - "sony_fms.job_pb2"
        - "sony_fms.job_pb2_grpc"
        - "sony_fms.fleet_pb2"
        - "sony_fms.fleet_pb2_grpc"
        - "sony_fms.common_header_pb2"
        - "sony_fms.common_geometry_pb2"
      agent_id_by_robot_name:
        robot_01: "robot_01"
      navigation:
        map_id_default: 1
        required_tag_by_waypoint:
          # RMF nav_graph waypoint_name -> "map_id/node_id"
          wp_A: "1/nodeA"
          charger: "1/charger"
        required_tag_format: "{map_id}/{waypoint}"  # fallback
      monitor_fleet_state:
        enable: true
        interval_sec: 1.0

    重要:
    - proto の python 生成物（pb2/pb2_grpc）は別途配置が必要
    - サービス名（Stub）は module から自動探索するが、複数サービス構成の場合は
      rpc 名で推定する（MonitorFleetState / RegisterJobs / ...）
    """

    def __init__(self, sony_cfg: Mapping[str, Any]) -> None:
        self._cfg = sony_cfg
        self._lock = threading.Lock()

        # job tracking: robot_name -> job_id
        self._active_move_job: Dict[str, str] = {}

        # fleet state cache: agent_id -> (x,y,theta_deg,battery_soc)
        self._agent_pose_cache: Dict[str, Tuple[float, float, float]] = {}
        self._agent_battery_cache: Dict[str, float] = {}

        # resolve config
        grpc_cfg = dict(sony_cfg.get("grpc", {}))
        self._target: str = str(grpc_cfg.get("target", "")).strip()
        if not self._target:
            raise ValueError("sony.grpc.target is required (e.g., '127.0.0.1:50051')")
        self._secure: bool = bool(grpc_cfg.get("secure", False))
        self._metadata: List[Tuple[str, str]] = []
        md = grpc_cfg.get("metadata", {}) or {}
        if isinstance(md, Mapping):
            for k, v in md.items():
                if v is None:
                    continue
                self._metadata.append((str(k), str(v)))

        # proto modules load
        self._pb2_modules: List[Any] = []
        self._pb2_grpc_modules: List[Any] = []
        for mod in sony_cfg.get("proto_modules", []) or []:
            m = importlib.import_module(str(mod))
            # heuristic: grpc module has "Stub" classes and "add_*Servicer_to_server"
            if hasattr(m, "__dict__") and any(k.endswith("Stub") for k in m.__dict__.keys()):
                self._pb2_grpc_modules.append(m)
            else:
                self._pb2_modules.append(m)

        # gRPC channel + stubs
        self._grpc = importlib.import_module("grpc")
        self._channel = self._create_channel(self._target, self._secure)

        # find stubs by required rpc names
        self._fleet_stub = self._find_stub(required_rpcs=("MonitorFleetState", "GetState", "ListAgents", "GetFleetState"))
        self._job_stub = self._find_stub(required_rpcs=("RegisterJobs", "StartJobs", "GetState", "CancelJobs", "PauseAgent", "ResumeAgent"))

        # state monitor
        mon_cfg = dict(sony_cfg.get("monitor_fleet_state", {}) or {})
        self._monitor_enable = bool(mon_cfg.get("enable", True))
        self._monitor_interval_sec = float(mon_cfg.get("interval_sec", 1.0))
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        if self._monitor_enable and self._fleet_stub is not None:
            self._thread = threading.Thread(target=self._fleet_monitor_loop, daemon=True)
            self._thread.start()

    # ----------------------------
    # plumbing
    # ----------------------------
    def _create_channel(self, target: str, secure: bool):
        if not secure:
            return self._grpc.insecure_channel(target)
        # TLS 証明書まわりは環境によって異なるため、ここは最小実装にする
        creds = self._grpc.ssl_channel_credentials()
        return self._grpc.secure_channel(target, creds)

    def _find_stub(self, required_rpcs: Iterable[str]):
        required = set(required_rpcs)
        for m in self._pb2_grpc_modules:
            for name, obj in getattr(m, "__dict__", {}).items():
                if not name.endswith("Stub") or not callable(obj):
                    continue
                try:
                    stub = obj(self._channel)
                except Exception:
                    continue
                # does stub have required RPC attributes?
                if all(hasattr(stub, rpc) for rpc in required):
                    return stub
        # fallback: any stub that has at least one rpc
        for m in self._pb2_grpc_modules:
            for name, obj in getattr(m, "__dict__", {}).items():
                if name.endswith("Stub") and callable(obj):
                    try:
                        return obj(self._channel)
                    except Exception:
                        continue
        return None

    def _find_msg_cls(self, cls_name: str):
        for m in self._pb2_modules:
            if hasattr(m, cls_name):
                return getattr(m, cls_name)
        return None

    def _new_msg(self, cls_name: str):
        cls = self._find_msg_cls(cls_name)
        if cls is None:
            raise RuntimeError(f"protobuf message '{cls_name}' not found. Check sony.proto_modules.")
        return cls()

    def _call(self, stub: Any, method_name: str, request: Any):
        if stub is None:
            raise RuntimeError(f"gRPC stub is not available for {method_name}")
        fn = getattr(stub, method_name, None)
        if fn is None:
            raise RuntimeError(f"RPC '{method_name}' not found on stub {type(stub).__name__}")
        if self._metadata:
            return fn(request, metadata=self._metadata)
        return fn(request)

    # ----------------------------
    # config helpers
    # ----------------------------
    def _agent_id(self, robot_name: str) -> str:
        m = self._cfg.get("agent_id_by_robot_name", {}) or {}
        if isinstance(m, Mapping) and robot_name in m:
            return str(m[robot_name])
        # fallback: assume same
        return robot_name

    def _required_tag(self, waypoint_name: Optional[str]) -> Optional[str]:
        if not waypoint_name:
            return None
        nav = self._cfg.get("navigation", {}) or {}
        by_wp = nav.get("required_tag_by_waypoint", {}) or {}
        if isinstance(by_wp, Mapping) and waypoint_name in by_wp:
            return str(by_wp[waypoint_name])
        map_id = nav.get("map_id_default", 1)
        fmt = nav.get("required_tag_format", "{map_id}/{waypoint}")
        try:
            return str(fmt).format(map_id=map_id, waypoint=waypoint_name)
        except Exception:
            return None

    # ----------------------------
    # monitoring
    # ----------------------------
    def _fleet_monitor_loop(self) -> None:
        # MonitorFleetState は server-streaming。接続不安定時は再接続を試みる。
        req = None
        try:
            req = self._new_msg("MonitorFleetStateRequest")
            # interval の型が google.protobuf.Duration のため、秒指定で設定できるなら設定する
            if hasattr(req, "interval"):
                try:
                    # Duration は seconds/nanos の message の場合が多い
                    req.interval.seconds = int(self._monitor_interval_sec)
                    req.interval.nanos = int((self._monitor_interval_sec - int(self._monitor_interval_sec)) * 1e9)
                except Exception:
                    pass
        except Exception:
            req = None

        while not self._stop_evt.is_set():
            try:
                if req is None:
                    # request を作れない場合は監視を諦める
                    time.sleep(1.0)
                    continue

                stream = self._call(self._fleet_stub, "MonitorFleetState", req)
                for resp in stream:
                    if self._stop_evt.is_set():
                        break
                    self._ingest_fleet_state_response(resp)
            except Exception:
                # 無限ループでのログ出力は騒がしいので抑制（必要なら FleetAdapter 側でログ）
                time.sleep(1.0)

    def _ingest_fleet_state_response(self, resp: Any) -> None:
        # resp.agents: AgentState[]
        agents = getattr(resp, "agents", None)
        if not agents:
            return

        for a in agents:
            try:
                agent_info = getattr(a, "agent_info", None)
                agent_id = getattr(agent_info, "id", None) if agent_info is not None else None
                if not agent_id:
                    continue

                nav_state = getattr(a, "navigation_state", None)
                pose = getattr(nav_state, "current_pose", None) if nav_state is not None else None
                x, y, theta_deg = self._pose_to_xytheta_deg(pose)

                sys_state = getattr(a, "system_state", None)
                battery = getattr(sys_state, "battery", None) if sys_state is not None else None
                soc = self._battery_to_soc(battery)

                with self._lock:
                    self._agent_pose_cache[str(agent_id)] = (x, y, theta_deg)
                    if soc is not None:
                        self._agent_battery_cache[str(agent_id)] = soc
            except Exception:
                continue

    @staticmethod
    def _battery_to_soc(battery_msg: Any) -> Optional[float]:
        if battery_msg is None:
            return None
        pct = getattr(battery_msg, "percentage", None)
        if pct is None:
            return None
        try:
            # percentage は 0-100 を想定
            return max(0.0, min(1.0, float(pct) / 100.0))
        except Exception:
            return None

    @staticmethod
    def _pose_to_xytheta_deg(pose_msg: Any) -> Tuple[float, float, float]:
        if pose_msg is None:
            return (0.0, 0.0, 0.0)

        pos = getattr(pose_msg, "position", None)
        x = float(getattr(pos, "x", 0.0)) if pos is not None else 0.0
        y = float(getattr(pos, "y", 0.0)) if pos is not None else 0.0

        # quaternion -> yaw
        q = getattr(pose_msg, "orientation", None)
        if q is None:
            return (x, y, 0.0)

        qx = float(getattr(q, "x", 0.0))
        qy = float(getattr(q, "y", 0.0))
        qz = float(getattr(q, "z", 0.0))
        qw = float(getattr(q, "w", 1.0))

        # yaw (Z) from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (x, y, math.degrees(yaw))

    # ----------------------------
    # API
    # ----------------------------
    def check_connection(self) -> bool:
        return True

    def position(self, robot_name: str) -> Optional[List[float]]:
        agent_id = self._agent_id(robot_name)
        with self._lock:
            if agent_id in self._agent_pose_cache:
                x, y, theta_deg = self._agent_pose_cache[agent_id]
                return [x, y, theta_deg]
        # fallback: try unary GetState if available
        try:
            req = self._new_msg("GetStateRequest")
            resp = self._call(self._fleet_stub, "GetState", req)
            self._ingest_fleet_state_response(resp)
            with self._lock:
                if agent_id in self._agent_pose_cache:
                    x, y, theta_deg = self._agent_pose_cache[agent_id]
                    return [x, y, theta_deg]
        except Exception:
            return None
        return None

    def navigate(
        self,
        robot_name: str,
        destination: List[float],
        map_name: str,
        *,
        waypoint_name: Optional[str] = None,
        angle_rad: Optional[float] = None,
        **_: Any,
    ) -> bool:
        _ = (destination, map_name)  # Sony は座標ではなく required_tag を基本にする

        required_tag = self._required_tag(waypoint_name)
        if not required_tag:
            return False

        agent_id = self._agent_id(robot_name)
        job_id = f"rmf_move_{robot_name}_{uuid.uuid4().hex}"

        try:
            # messages
            MoveJob = self._find_msg_cls("MoveJob")
            PositionRequirement = self._find_msg_cls("PositionRequirement")
            PositionCandidate = self._find_msg_cls("PositionCandidate")
            AgentRequirement = self._find_msg_cls("AgentRequirement")
            AgentTargets = self._find_msg_cls("AgentTargets")
            RegisterJobsRequest = self._find_msg_cls("RegisterJobsRequest")
            StartJobsRequest = self._find_msg_cls("StartJobsRequest")
            JobFilter = self._find_msg_cls("JobFilter")

            if None in (MoveJob, PositionRequirement, PositionCandidate, AgentRequirement, AgentTargets,
                        RegisterJobsRequest, StartJobsRequest, JobFilter):
                raise RuntimeError("required protobuf messages are missing. Check sony.proto_modules.")

            # Build request
            cand = PositionCandidate(required_tags=[required_tag])
            if angle_rad is not None:
                # Sony spec: angle is rad in map coordinates
                cand.angle = float(angle_rad)

            pos_req = PositionRequirement(candidates=[cand])
            agent_targets = AgentTargets(ids=[agent_id])
            agent_req = AgentRequirement(targets=agent_targets)

            move_job = MoveJob(job_id=job_id, position=pos_req, agent=agent_req, priority=0)

            reg_req = RegisterJobsRequest(move_jobs=[move_job])
            self._call(self._job_stub, "RegisterJobs", reg_req)

            start_req = StartJobsRequest(job_filter=JobFilter(job_ids=[job_id]))
            self._call(self._job_stub, "StartJobs", start_req)

            with self._lock:
                self._active_move_job[robot_name] = job_id
            return True

        except Exception:
            return False

    def stop(self, robot_name: str) -> bool:
        with self._lock:
            job_id = self._active_move_job.get(robot_name)

        if not job_id:
            return True

        try:
            CancelJobsRequest = self._find_msg_cls("CancelJobsRequest")
            if CancelJobsRequest is None:
                return False
            req = CancelJobsRequest(job_ids=[job_id])
            self._call(self._job_stub, "CancelJobs", req)
            return True
        except Exception:
            return False

    def navigation_completed(self, robot_name: str) -> bool:
        with self._lock:
            job_id = self._active_move_job.get(robot_name)
        if not job_id:
            return True

        try:
            GetStateRequest = self._find_msg_cls("GetStateRequest")
            JobFilter = self._find_msg_cls("JobFilter")
            if GetStateRequest is None or JobFilter is None:
                return False

            req = GetStateRequest(job_filter=JobFilter(job_ids=[job_id]))
            resp = self._call(self._job_stub, "GetState", req)
            state = getattr(resp, "state", None)
            move_states = getattr(state, "move_job_state", None) if state is not None else None
            if not move_states:
                return False

            # enum value compare: MOVE_STATUS_COMPLETED or MOVE_STATUS_FAILED 等
            # python 生成物では数値(enum)で入っていることがあるため、名前解決はしない
            for ms in move_states:
                if getattr(ms, "job_id", "") != job_id:
                    continue
                status_val = getattr(ms, "status", None)
                # Completed = 5, Failed = 7, Canceled = 6, Paused = 8 (MoveStatus.md の列挙順)
                if status_val in (5, 6, 7):
                    with self._lock:
                        self._active_move_job.pop(robot_name, None)
                    return True
                if status_val == 5:
                    return True
            return False
        except Exception:
            return False

    def start_process(self, robot_name: str, process: str, map_name: str) -> bool:
        _ = (process, map_name)
        # Sony 側の「設備連携」や「TransportJob」に寄せる場合、ここを拡張する。
        # テンプレでは docking を process として扱うため、未対応なら no-op で成功とする。
        return True

    def process_completed(self, robot_name: str) -> bool:
        _ = robot_name
        return True

    def pause(self, robot_name: str) -> bool:
        agent_id = self._agent_id(robot_name)
        try:
            PauseAgentRequest = self._find_msg_cls("PauseAgentRequest")
            AgentRequirement = self._find_msg_cls("AgentRequirement")
            AgentTargets = self._find_msg_cls("AgentTargets")
            if None in (PauseAgentRequest, AgentRequirement, AgentTargets):
                return False
            req = PauseAgentRequest(agent=AgentRequirement(targets=AgentTargets(ids=[agent_id])))
            self._call(self._job_stub, "PauseAgent", req)
            return True
        except Exception:
            return False

    def resume(self, robot_name: str) -> bool:
        agent_id = self._agent_id(robot_name)
        try:
            ResumeAgentRequest = self._find_msg_cls("ResumeAgentRequest")
            AgentRequirement = self._find_msg_cls("AgentRequirement")
            AgentTargets = self._find_msg_cls("AgentTargets")
            if None in (ResumeAgentRequest, AgentRequirement, AgentTargets):
                return False
            req = ResumeAgentRequest(agent=AgentRequirement(targets=AgentTargets(ids=[agent_id])))
            self._call(self._job_stub, "ResumeAgent", req)
            return True
        except Exception:
            return False

    def battery_soc(self, robot_name: str) -> Optional[float]:
        agent_id = self._agent_id(robot_name)
        with self._lock:
            if agent_id in self._agent_battery_cache:
                return self._agent_battery_cache[agent_id]
        # if no cache, fallback to position() which may ingest state
        _ = self.position(robot_name)
        with self._lock:
            return self._agent_battery_cache.get(agent_id)

    def shutdown(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self._channel.close()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# Public facade used by FleetAdapter
# -----------------------------------------------------------------------------
class RobotAPI:
    """FleetAdapter から呼ばれる facade。

    - デフォルトは _SimBackend
    - config.yaml に sony セクションが存在する場合は _SonyGrpcBackend を選択
    """

    def __init__(self, prefix: str, username: str, password: str, *, extra_config: Optional[Mapping[str, Any]] = None):
        self.prefix = prefix
        self.username = username
        self.password = password
        self.connected = False

        sony_cfg = None
        if extra_config and isinstance(extra_config, Mapping):
            sony_cfg = extra_config.get("sony")

        if isinstance(sony_cfg, Mapping):
            # Sony を有効化
            self._backend: _BackendBase = _SonyGrpcBackend(sony_cfg)
        else:
            # それ以外はシミュレーション
            self._backend = _SimBackend()

        self.connected = True

    # ---------------------------------------------------------------------
    # API expected by template (delegation)
    # ---------------------------------------------------------------------
    def check_connection(self) -> bool:
        return self._backend.check_connection()

    def position(self, robot_name: str) -> Optional[List[float]]:
        return self._backend.position(robot_name)

    def navigate(
        self,
        robot_name: str,
        destination: List[float],
        map_name: str,
        *,
        waypoint_name: Optional[str] = None,
        angle_rad: Optional[float] = None,
        **kwargs: Any,
    ) -> bool:
        return self._backend.navigate(
            robot_name,
            destination,
            map_name,
            waypoint_name=waypoint_name,
            angle_rad=angle_rad,
            **kwargs,
        )

    def stop(self, robot_name: str) -> bool:
        return self._backend.stop(robot_name)

    def navigation_remaining_duration(self, robot_name: str) -> float:
        return self._backend.navigation_remaining_duration(robot_name)

    def navigation_completed(self, robot_name: str) -> bool:
        return self._backend.navigation_completed(robot_name)

    def start_process(self, robot_name: str, process: str, map_name: str) -> bool:
        return self._backend.start_process(robot_name, process, map_name)

    def process_completed(self, robot_name: str) -> bool:
        return self._backend.process_completed(robot_name)

    def docking_completed(self, robot_name: str) -> bool:
        return self._backend.docking_completed(robot_name)

    def pause(self, robot_name: str) -> bool:
        return self._backend.pause(robot_name)

    def resume(self, robot_name: str) -> bool:
        return self._backend.resume(robot_name)

    def battery_soc(self, robot_name: str) -> Optional[float]:
        return self._backend.battery_soc(robot_name)

    def shutdown(self) -> None:
        self._backend.shutdown()
