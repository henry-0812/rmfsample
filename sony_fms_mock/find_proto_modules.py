#!/usr/bin/env python3
# find_proto_modules.py
#
# 「*_pb2.py / *_pb2_grpc.py がどこにあるか」を探し、import 用の module 名の候補を出す補助スクリプト。
#
# 使い方:
#   python3 find_proto_modules.py --root /root/rmf_ws
#
# 出力:
#   - 見つかった *_pb2_grpc.py のパス一覧
#   - "python -c 'import <module>'" の形で試せる候補文字列
#
from __future__ import annotations

import argparse
import os
import re
from pathlib import Path
from typing import Iterable, List, Tuple


def _path_to_module_candidates(file_path: Path, roots: List[Path]) -> List[str]:
    """
    file_path を roots 配下の相対パスに変換できた場合、
    a/b/c_pb2_grpc.py -> a.b.c_pb2_grpc のような候補を返す。
    """
    candidates: List[str] = []
    for r in roots:
        try:
            rel = file_path.relative_to(r)
        except Exception:
            continue
        # __init__.py が無い階層もあるので「候補」として出すだけにする
        mod = ".".join(rel.with_suffix("").parts)
        candidates.append(mod)
    return candidates


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="検索起点ディレクトリ（例: /root/rmf_ws）")
    args = ap.parse_args()

    root = Path(args.root)
    if not root.exists():
        raise SystemExit(f"root not found: {root}")

    # よくある Python パッケージルート候補
    roots = [
        root,
        root / "install",
        root / "src",
        root / "build",
        Path("/usr/lib/python3/dist-packages"),
        Path("/usr/local/lib/python3/dist-packages"),
        Path("/opt/ros"),
    ]

    pb2_grpc_files = sorted(root.rglob("*_pb2_grpc.py"))
    pb2_files = sorted(root.rglob("*_pb2.py"))

    print("=== Found *_pb2_grpc.py ===")
    for p in pb2_grpc_files[:200]:
        print(str(p))
    if len(pb2_grpc_files) > 200:
        print(f"... ({len(pb2_grpc_files)} total)")

    print("\n=== Found *_pb2.py (first 50) ===")
    for p in pb2_files[:50]:
        print(str(p))
    if len(pb2_files) > 50:
        print(f"... ({len(pb2_files)} total)")

    print("\n=== Import module candidates (for *_pb2_grpc.py) ===")
    seen = set()
    for p in pb2_grpc_files[:200]:
        for cand in _path_to_module_candidates(p, roots):
            if cand in seen:
                continue
            seen.add(cand)
            print(cand)

    print("\nHint:")
    print("  上の候補を 1つずつ試すなら例:")
    print("    python3 -c \"import <module_name>; print('OK')\"")


if __name__ == "__main__":
    main()
