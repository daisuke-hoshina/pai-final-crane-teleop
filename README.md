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
2. C を実行し、「記録中」と出たらすぐ A で操作する
3. **D を押しっぱなし**にして右へ回り、B の「回る」が 0.5cm 以内で離す（ブロックはホームと同じく根元から 10cm にあるので、回るだけで真上に着く）
4. **F を押しっぱなし**にして、一番下（手先が机から 3.5cm、`too low`、B に「★ つかむ位置」）まで下ろす
5. すぐ C で閉じて 1 秒待ち、**R を押しっぱなし**にして速く持ち上げる（ゆっくり上げると、上がるより滑る方が速い）
6. B に「★ 持ち上げ成功」と出て約 1 秒で R を離し、すぐ C で Ctrl+C。長さ・記録中の最大の浮き・成功したかが表示される

キーは押しっぱなしで動かす。1 回ずつ押すと、押す合間に腕が止まった区間がデモに入る（学習した方策がそこで止まる原因になる）。

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
cd crane_ee_teleop && python3 -m pytest test/
```

（作業中。学習・評価の結果は随時追記する）
