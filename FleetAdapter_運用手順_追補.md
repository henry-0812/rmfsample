# FleetAdapter 運用手順（追補）

## 1. パッケージのビルド（デプロイ）

### 1.1 パッケージ名の確認
```bash
cat /root/rmf_ws/src/origin/<pkg_name>/package.xml | grep "<name>"
```

### 1.2 ビルド（対象パッケージのみ）
```bash
PKG="<pkg_name>"
cd /root/rmf_ws
rm -rf "/root/rmf_ws/install/$PKG" "/root/rmf_ws/build/$PKG"
source /opt/ros/jazzy/setup.bash
source /root/rmf_ws/install/setup.bash
colcon build --packages-select "$PKG"
```

---

## 2. ノードの起動（FleetAdapter）

### 2.1 `ros2 run` で起動
```bash
ros2 run <pkg_name> <executable_name>   -c /root/rmf_ws/src/origin/<pkg_name>/<pkg_name>/config.yaml   -n /root/rmf_ws/install/rmf_launches/share/rmf_launches/config/mock/<nav_graph>.yaml   --use_sim_time
```

### 2.2 `ros2 launch` で起動
```bash
ros2 launch <pkg_name> <launch_file>.launch.py   config_file:=/root/rmf_ws/src/origin/<pkg_name>/<pkg_name>/config.yaml   nav_graph:=/root/rmf_ws/install/rmf_launches/share/rmf_launches/config/mock/<nav_graph>.yaml   use_sim_time:=true
```

---

## 3. 修正の反映確認（最小）

### 3.1 FleetAdapter の再起動
- FleetAdapter を停止して、同じ起動コマンドで再起動する。

### 3.2 再ビルドが必要なケース
- `setup.py` / `package.xml` / `entry_points` / `launch.py` / 依存関係の変更  
  → 「1. パッケージのビルド（デプロイ）」を実行してから再起動する。

---

## 4. Sony gRPC モックの使用手順（FleetManager 無しで API 実行確認）

### 4.1 モックの展開
```bash
mkdir -p /root/sony_fms_mock
cd /root/sony_fms_mock
unzip sony_fms_mock.zip
```

### 4.2 依存インストール
```bash
python3 -m pip install -r /root/sony_fms_mock/requirements.txt
```

### 4.3 `config.yaml` の設定
`/root/rmf_ws/src/origin/<pkg_name>/<pkg_name>/config.yaml` に以下を設定する。

- `sony.grpc.target` をモックへ向ける（例）
```yaml
sony:
  grpc:
    target: "127.0.0.1:50051"
```

- `sony.proto_modules` に `*_pb2.py / *_pb2_grpc.py` の import 可能な module 名を列挙する
```yaml
sony:
  proto_modules:
    - <module>_pb2
    - <module>_pb2_grpc
```

### 4.4 `proto_modules` の探索（必要な場合）
```bash
find /root/rmf_ws -name "*_pb2_grpc.py" -o -name "*_pb2.py" | head -n 50
python3 /root/sony_fms_mock/find_proto_modules.py --root /root/rmf_ws
python3 -c "import <candidate_module>; print('OK')"
```

### 4.5 モック起動
```bash
python3 /root/sony_fms_mock/sony_fms_mock_server.py   --config /root/rmf_ws/src/origin/<pkg_name>/<pkg_name>/config.yaml
```

### 4.6 FleetAdapter 起動
「2. ノードの起動（FleetAdapter）」の手順で起動する。

### 4.7 API 実行確認
- モック側標準出力に RPC 名が出力されることを確認する。

---

## 5. 確認コマンド

### 5.1 ノード一覧
```bash
ros2 node list
```

### 5.2 トピック一覧（候補抽出）
```bash
ros2 topic list | egrep -i "fleet|rmf|task|schedule|robot|state|clock"
```

### 5.3 sim time 確認
```bash
ros2 topic echo /clock --once
```
