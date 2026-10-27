# pai-final-crane-teleop

Physical AI 応用編1 最終課題（Daisuke-Hoshina）。

Gazebo 上の CRANE+ V2 に、キーボードで手先を動かす「手先空間テレオペ」を作り、
それで集めたデモから模倣学習（ACT）で「机の上のブロックをつかんで持ち上げる」方策を学習する。
既存の関節空間テレオペとの操作のしやすさの比較も行う。

ベースにしたサンプル: 「知能ロボットシステムエンジニアリング第5回 ROS 2入門（2/2）」
（CRANE+ のテレオペ → rosbag 記録 → LeRobot 形式へ変換 → ACT 学習 → 推論）。

## 構成

- `crane_ee_teleop/` … ROS 2 パッケージ（ament_python）
  - `crane_ee_teleop/kinematics.py` … CRANE+ V2 の順運動学・逆運動学（解析解。ROS に依存しない）
  - `crane_ee_teleop/teleop_keyboard_ee.py` … 手先空間キーボードテレオペ。
    送るトピック・形式・周期（20Hz）は `physicalai_teleop` と同じ
  - `crane_ee_teleop/teleop_keyboard_joint.py` … 関節空間キーボードテレオペ（比較の対照群）。
    `physicalai_teleop` のコピーで、変更はグリッパーの `time_from_start`（0.1 → 0.01 秒）と
    X で戻る姿勢（直立 → 手先空間テレオペと同じホーム）だけ。比較で変える条件をテレオペ方式だけにする
  - `crane_ee_teleop/task.py` … タスクの定義（grasp: つかむ位置＝ブロックの中心の 1cm 上、成功＝一度でも 3cm 以上浮く。
    reach: 目標点＝ブロックの中心の真上 5.5cm、成功＝最後に 1cm 以内）
  - `crane_ee_teleop/task_monitor.py` … 手先から目標点までの差と、ブロックの浮きを表示する（指令は送らない）
  - `crane_ee_teleop/record_episode.py` … デモを 1 エピソード記録する（ホームの確認、番号の自動採番、成功の判定）
  - `crane_ee_teleop/gazebo_objects.py` … Gazebo のモデル（ブロック）の位置を読む・初期位置へ戻す
- `policy/` … 模倣学習（変換・学習設定・推論・評価）。下の「模倣学習（ACT）」

## Gazebo の準備（指の摩擦を上げる）

講義の環境のままでは、グリッパーの指の摩擦係数が 0.2 と低く、つかんだブロックが持ち上げる間に滑り落ちる。
このリポジトリのデモと評価は、指（`crane_plus_link4` と `crane_plus_link_hand`）の `mu1`・`mu2` を **10.0** に上げた環境で行った。
講義の `crane_plus_description` パッケージの `urdf/crane_plus.gazebo.xacro` で、この 2 つのリンクの `<gazebo reference=...>` の
`<mu1>0.2</mu1>` `<mu2>0.2</mu2>` を `10.0` に書き換え、Gazebo を起動し直す（symlink install なら再ビルドは不要）。

確かめ方:

```bash
ros2 param get /robot_state_publisher robot_description | grep -o "<mu1>[0-9.]*</mu1>" | sort | uniq -c
#   4 <mu1>0.2</mu1>
#   2 <mu1>10.0</mu1>
```

摩擦を上げても、持ち上げたブロックは 1 秒に約 1cm ずつ滑り、数秒で落ちる（摩擦 1.0 / 2.0 / 10 で大きな差はなかった）。
そのため成功は「一度でも 3cm 以上浮いた」で判定し、持ち上げは速く行う。

## 手先空間テレオペの使い方

Gazebo は MoveIt・RViz なしで起動する（テレオペには不要で、重いため。実時間比 66% → 81%）:

```bash
ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py
ros2 run crane_ee_teleop teleop_keyboard_ee
```

| キー | 動作 |
|---|---|
| A / D | 根元のまわりに左 / 右へ回る（根元からの距離と高さはそのまま。joint1 だけが動く。1 回 0.05 rad、根元から 10cm で約 5mm） |
| W / S | 今の向きと高さのまま、根元から遠く / 近くへ（5mm ずつ） |
| R / F | 今の向きと根元からの距離のまま、上 / 下へ（5mm ずつ） |
| O / C | グリッパーを開く / 閉じる |
| X | ホームへ（机の上 10cm、手先はほぼ真下向き） |
| Z | ブロックを初期位置へ戻す（記録を始める前だけ使う） |
| ESC | 終了 |

- 手先の位置は、根元を中心にした向き・距離・高さで動かす。2026-10-27 まではまっすぐの前後・左右（x, y）だったが、
  A/D で右へ動かすと手先が根元から遠ざかって腕が伸びる（10cm → 14cm）ため変えた。下の「関節空間テレオペとの比較」はまっすぐ版で行った
- 手先の向きは操作しない。いつも「その位置で届く範囲でいちばん真下に近い向き」に自動で決まる
- 手先は机から 3.2cm より下へは下げない。手先点は指先の 2〜3cm 上にあり、それより下では指先が机に着く
  （着いたまま下げる指令を送り続けると、腕が机の上を滑って外へ伸びる）
- 関節の可動範囲を越える位置や、1 回で関節が大きく跳ぶ位置へは動かない（"cannot move there"）

## タスクとデモの記録

タスク（grasp）: ホーム（X）から、ブロックをつかんで持ち上げる。エピソード中に一度でもブロックの底面が机から 3cm 以上浮けば成功。

ターミナルを 3 つ使う:

```bash
ros2 run crane_ee_teleop teleop_keyboard_ee            # A: テレオペ
ros2 run crane_ee_teleop task_monitor                  # B: つかむ位置までの差と、ブロックの浮きを表示
ros2 run crane_ee_teleop record_episode --method ee    # C: 1 エピソード記録（--root で保存先の親フォルダを変えられる）
```

B は差をテレオペのキーと同じ向きで出す（回る / 遠く近く / 上下。合わせるキーが横に出る）。

1. A で X（ホーム）、止まったら Z（ブロックを初期位置へ）
2. C を実行し、「記録中」と出たら **2 秒数えてから** A で操作する（記録の書き込みは表示より少し遅れて始まる。すぐ動かすと「ホームから始まっていません」になる）
3. **D をちょうど 31 回**、数えながら一定のリズムで押して右へ回る（1 回 2.9°、31 回で 89°。腕は指令より約 1 秒遅れるので、腕を見ながら押すと回りすぎる）。
   ブロックはホームと同じく根元から 10cm にあるので、回るだけで真上に着く
4. **F** でまっすぐ下ろし、一番下（`too low`、B に「★ つかむ位置」）で止める
5. 一番下ですぐ C キーで閉じて 1 秒待ち、**R を押しっぱなし**にして速く持ち上げる（1 回ずつだと、上がるより滑る方が速くブロックが抜ける）
6. B に「★ 持ち上げ成功」と出て約 1 秒で R を離し、ターミナル C で Ctrl+C。長さ・記録中の最大の浮き・成功したかが表示される

途中で止まって考えない。止まった区間がデモに入ると、学習した方策がそこで止まる原因になる。
提出の方策（下の「模倣学習（ACT）」）は、この手順で録った成功 13 本のうち、録った順に最初の 10 本で学習した。デモの rosbag はリポジトリに入れていない。

記録は既定で `~/dataset/rosbags/grasp_ee/epN`。変換は `pai_training` の `rosbag2lerobot.py` をそのまま使える。
`--task reach` を付けると、以前のタスク（手先をブロックの真上 5.5cm へ運ぶ）になる。

## 関節空間テレオペとの比較

（手先空間テレオペがまっすぐの前後・左右（x, y）で動いていた版、コミット `826340d` で行った）

`teleop_keyboard_joint` は講義の `physicalai_teleop` のコピー（変更はグリッパーの `time_from_start` と X で戻る姿勢だけ）。
reach タスクで、手先空間テレオペは 16 回中 16 回成功（慣れて 1 本 40 秒台）、関節空間テレオペは 3 回とも届かず断念した。
主な理由: R/F は選んだ関節ではなく全関節を動かす、指令値が関節の可動範囲で止まらず押しても動かない区間ができる、
空間の 1 点へ手先を運ぶには 3〜4 関節を同時に合わせる必要がある。

## テスト

```bash
source /opt/ros/humble/setup.bash   # task_monitor などのテストは rclpy を import する
cd crane_ee_teleop && python3 -m pytest test/
```

## 模倣学習（ACT）

講義の演習用リポジトリ [matsuolab/pai_training](https://github.com/matsuolab/pai_training)（commit `b4d6b54`）の上で動かす。
`policy/` のうち、変換スクリプト・推論スクリプト・設定ファイルはそのリポジトリのファイルを写して変えたもので、各ファイルの先頭に出典を書いた。
`eval_policy.py` は自作。

- `policy/convert/` … rosbag → LeRobot 形式の変換（元は `scripts/rosbag2lerobot.py`）。講義版からの変更:
  パディングをしない、腕が止まっている区間を 4 フレーム（0.2 秒）まで詰める、
  `observation.environment_state` を関節角の複製から「0.1 秒前からの関節角の変化（速さ）」にする
- `policy/train/configs/` … 学習設定（講義の `scripts/train.py` をそのまま使う）。講義版からの変更は batch_size 16・training_steps 10,000 とデータの場所
- `policy/infer_chunk.py` … 推論（元は `scripts/infer.py`）。オプションを付けなければ講義版と同じ動き。
  提出の構成では `--vel-env`（観測に速さ）と `--ensemble 0.01`（時間的アンサンブル: 0.1 秒ごとに予測し、
  過去の予測のうち今の時刻に当たる指令を平均する。ACT の論文の推論）を使う
- `policy/eval_policy.py` … デプロイして N 回測る（ホームへ戻す → ブロックを初期位置へ → 推論を起動 → ブロックが 3cm 以上浮けば成功）

準備: 講義の手順で `pai_training` を `~/pai_training` に clone して `pixi install` する。このパッケージは ROS 2 のワークスペースに置いて
`colcon build --packages-select crane_ee_teleop` する。パスはコンテナ（ユーザー `ubuntu`、このリポジトリを `/workspace/pai-final-crane-teleop` に置く）の中を前提にしている。
デモの rosbag・変換したデータセット・学習した重みはリポジトリに入れていない（`conversion_config.yaml` の `bag_dir` を自分で録ったデモに向ける）。

```bash
# 1. 変換（デモ 10 本の rosbag → LeRobot 形式）
cd policy/convert && pixi run --manifest-path ~/pai_training/pyproject.toml python rosbag2lerobot_trimstill_vel.py
# 2. 学習（GPU MX350 で約 31 分）
cd policy/train && pixi run --manifest-path ~/pai_training/pyproject.toml python ~/pai_training/scripts/train.py
# 3. 評価（10 回、各 120 秒。Gazebo とコントローラを起動しておく）
source /opt/ros/humble/setup.bash && source ~/pai_ws/install/setup.bash
cd policy && python3 eval_policy.py ~/checkpoints/train/crane_plus_grasp_act_v2_vel_batch16 10 120 \
  "/workspace/pai-final-crane-teleop/policy/infer_chunk.py --vel-env --index 1 --hold 2 --ensemble 0.01"
```

結果: **10 回中 9 回成功**（ブロックの浮き 10.3〜18.6cm）。ルール（手で書いた条件）は使わず、方策の出力だけで
「ブロックの方へ回る → 真上で下りる → 一番下で閉じる → 持ち上げる」を行う。
講義の推論（毎ティック chunk の先頭だけを使う）では、同じモデルが回る途中で止まった（0/5）。
