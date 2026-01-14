# Sony FMS gRPC Mock（FleetAdapter API 実行確認用）

このZIPは **FleetManager（Sony FMS）が無い環境でも**、`fleet_adapter_sony` が叩く gRPC API を **実際に実行させて疎通確認**するための「最小モックサーバ」です。

- FleetAdapter 側の Python ソースは **変更しません**
- 必要なのは `config.yaml` の `sony:` セクションを有効化して、モック（localhost）へ向けるだけです

---

## 1. 何ができるか（ゴール）

このモックを起動すると、FleetAdapter から以下が確認できます。

### A) “Fleet状態同期” の疎通（起動しただけで起きる）
FleetAdapter が起動して最初に行うことの一つが **FleetState購読（stream）**です。

- `MonitorFleetState` が呼ばれる
- `GetState` が呼ばれる（環境や実装により）

これがモック側ログに出れば、**FleetAdapter → FleetManager(gRPC) の接続/呼び出しはできています**。

### B) “Job系API” の疎通（RMFから移動指令が出たときに起きる）
RMFからループタスク/移動指令が出ると、一般に以下が呼ばれます。

- `RegisterJobs`
- `StartJobs`
- `GetState`（ジョブ状態確認）
- `CancelJobs`（キャンセル時）
- `PauseAgent` / `ResumeAgent`（一時停止/再開がある場合）

モック側ログに `RegisterJobs` / `StartJobs` が出れば、**「タスク投入 → FleetAdapter → FleetManager呼び出し」**まで確認できます。

---

## 2. 前提（必要なもの）

### 2.1 Python 依存
モックは Python で動きます。必要パッケージ:

- `grpcio`
- `PyYAML`

Ubuntu で足りない場合は以下（例）:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install grpcio pyyaml
```

### 2.2 proto 生成物（最重要）
FleetAdapter が使う Sony の gRPC 定義（proto）から生成された

- `*_pb2.py`
- `*_pb2_grpc.py`

が **Python から import できる状態**である必要があります。

> 既に FleetAdapter が Sony proto を使う設計で作られている場合、どこかに生成物があるはずです。

---

## 3. 手順（丁寧版）：モック起動 → FleetAdapterから疎通確認

ここからは **“実際のコマンド実行順”** で説明します。

---

### Step 0: どこで動かす？
原則、FleetAdapter を動かしている **同じコンテナ内** でモックも起動するのが一番ラクです。

- FleetAdapter もモックも「127.0.0.1:50051」等でつながる
- Dockerのポート公開を考えなくてよい

---

### Step 1: ZIPを展開する
任意ディレクトリで展開します。

```bash
mkdir -p /root/sony_fms_mock
cd /root/sony_fms_mock
unzip sony_fms_mock.zip
```

ファイルは以下です。

- `sony_fms_mock_server.py` : モック本体
- `find_proto_modules.py`   : proto生成物の場所を探す補助（必要な場合）
- `README.md`               : この説明

---

### Step 2: config.yaml の sony セクションを有効化する（FleetAdapter側の作業）
`fleet_adapter_sony/config.yaml` に `sony:` ブロックがあるはずなので、コメントアウトされている場合は解除します。

#### 2.1 gRPC target をモックへ向ける
例：

```yaml
sony:
  grpc:
    target: "127.0.0.1:50051"
```

> ここは「モックが待ち受けるポート」です。次の手順でも同じ port を使います。

#### 2.2 proto_modules を設定する
`proto_modules:` は **生成された pb2/pb2_grpc の module 名**を列挙します。

例（※例なのであなたの環境の名前に合わせてください）:

```yaml
sony:
  proto_modules:
    - sony_fms.fleet_pb2
    - sony_fms.fleet_pb2_grpc
    - sony_fms.job_pb2
    - sony_fms.job_pb2_grpc
```

---

### Step 3: proto_modules が分からない場合の探し方（補助）

#### 3.1 まずファイルの場所を find で探す（確実）
```bash
find /root/rmf_ws -name "*_pb2_grpc.py" -o -name "*_pb2.py" | head
```

#### 3.2 候補をまとめて出す（補助スクリプト）
```bash
python3 /root/sony_fms_mock/find_proto_modules.py --root /root/rmf_ws
```

出てきた候補（例: `something.something_pb2_grpc`）を 1つずつ import テストします。

```bash
python3 -c "import something.something_pb2_grpc; print('OK')"
```

**OKが出た module 名**を `proto_modules` に書けばOKです。

---

### Step 4: モックサーバを起動する
`config.yaml` を読むので、**同じ config.yaml を渡して起動**します。

```bash
python3 /root/sony_fms_mock/sony_fms_mock_server.py \
  --config /root/rmf_ws/src/origin/fleet_adapter_sony/fleet_adapter_sony/config.yaml
```

起動するとこんなログが出ます。

- `[sony-fms-mock] listening on 0.0.0.0:50051 ...`

---

### Step 5: FleetAdapter を起動する（別ターミナル推奨）
（あなたの環境で動いているコマンドでOK）

例:

```bash
ros2 run fleet_adapter_sony fleet_adapter \
  -c /root/rmf_ws/src/origin/fleet_adapter_sony/fleet_adapter_sony/config.yaml \
  -n /root/rmf_ws/install/rmf_launches/share/rmf_launches/config/mock/irayple_mock_nav_graph.yaml \
  --use_sim_time
```

---

### Step 6: “Fleet同期の疎通”を確認する（起動直後に分かる）
モック側ターミナルを見ます。

FleetAdapter が接続すると、モックは RPC 名を出します。

例:

- `[sony-fms-mock][rpc] /sony.fleet.Fleet/MonitorFleetState`
- `[sony-fms-mock][rpc] /sony.fleet.Fleet/GetState`

このログが出たら、**FleetAdapter → FleetManager(gRPC) の接続＆API実行**が成功しています。

---

### Step 7: “Job系APIの疎通” を確認する（タスクを投げる）
次に、RMFから **ループタスク（loop）**を出してみてください。

- Web UI でループタスクを投げる（環境によってUIが違うので、あなたのいつもの手順でOK）
- もしくは、既に使っている “タスク投入” の流れでOK

タスクが FleetAdapter に届くと、モック側でこういう RPC が出ます。

- `RegisterJobs`
- `StartJobs`
- `GetState`

ここまで出れば **「RMF → FleetAdapter → FleetManager（モック）へ命令が飛んでいる」** が確認できます。

---

## 4. よくあるハマりポイント（原因→対処）

### 4.1 「proto_modules import 失敗」
症状:
- `ModuleNotFoundError: No module named ...`

対処:
- `find` で実際の `*_pb2_grpc.py` の場所を探し、module 名を正しくする
- `PYTHONPATH` にそのディレクトリを追加する（必要な場合）

例:
```bash
export PYTHONPATH=$PYTHONPATH:/root/rmf_ws/install/<some_pkg>/lib/python3.12/site-packages
```

### 4.2 FleetAdapter が gRPC を叩かない（Sim側に落ちている）
症状:
- モック側に RPC ログが何も出ない

対処:
- config.yaml の `sony:` を有効化したか確認
- FleetAdapter 側が sony backend を選ぶ条件（設定キー）になっているか確認
  - ここはあなたの `fleet_adapter_sony` の設計に依存します

---

## 5. 目標達成のチェックリスト

✅ モック起動ログに `listening on ...` が出る  
✅ FleetAdapter起動直後に、モック側に `MonitorFleetState` または `GetState` が出る  
✅ タスク投入で、モック側に `RegisterJobs` / `StartJobs` が出る

これが揃えば「APIの実行確認」は完了です。

---
