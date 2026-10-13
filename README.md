# pai-final-crane-teleop

Physical AI 応用編1 最終課題（Daisuke-Hoshina）。

Gazebo 上の CRANE+ V2 に、キーボードで手先を動かす「手先空間テレオペ」を作り、
既存の関節空間テレオペと比べて、模倣学習（ACT）のデモと方策がどう変わるかを調べる。

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
  - `crane_ee_teleop/grasp_monitor.py` … 手先からブロックまでの距離を表示する（指令は送らない。どちらのテレオペとも併用できる）
  - `crane_ee_teleop/gazebo_objects.py` … Gazebo のモデル（ブロック）の位置を読む

## 手先空間テレオペの使い方

Gazebo は MoveIt・RViz なしで起動する（テレオペには不要で、重いため。実時間比 66% → 81%）:

```bash
ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py
ros2 run crane_ee_teleop teleop_keyboard_ee
```

| キー | 動作 |
|---|---|
| W / S | 手先を前 / 後ろへ（5mm ずつ） |
| A / D | 手先を左 / 右へ |
| R / F | 手先を上 / 下へ |
| O / C | グリッパーを開く / 閉じる |
| X | ホームへ（机の上 10cm、手先はほぼ真下向き） |
| ESC | 終了 |

- 手先の向きは操作しない。いつも「その位置で届く範囲でいちばん真下に近い向き」に自動で決まる
- 関節空間テレオペ（`teleop_keyboard_joint`）の X も同じホームに戻る

別のターミナルで、手先からブロックまでの距離を表示できる:

```bash
ros2 run crane_ee_teleop grasp_monitor
```

## テスト

```bash
cd crane_ee_teleop && python3 -m pytest test/
```

（作業中。実行方法・結果は随時追記する）
