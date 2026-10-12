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
  - `crane_ee_teleop/grasp_monitor.py` … 手先からブロックまでの距離を表示する（指令は送らない。どちらのテレオペとも併用できる）
  - `crane_ee_teleop/gazebo_objects.py` … Gazebo のモデル（ブロック）の位置を読む

## 手先空間テレオペの使い方

```bash
ros2 run crane_ee_teleop teleop_keyboard_ee
```

| キー | 動作 |
|---|---|
| W / S | 手先を前 / 後ろへ（+x / -x、5mm ずつ） |
| A / D | 手先を左 / 右へ（+y / -y） |
| R / F | 手先を上 / 下へ（+z / -z） |
| T / G | 手先を下向き / 上向きに傾ける（約 3° ずつ） |
| V | いまの位置のまま、届く範囲で最も真下向きに近い傾きにする |
| O / C | グリッパーを開く / 閉じる |
| X | 直立（全関節 0）に戻す |
| ESC | 終了 |

- 位置を動かすたびに、手先の傾きを真下向きへ 1 段（約 3°）近づける。ブロックを上からつかむため
- 位置を動かして届かないときは、傾きを自動で少しずつ変えて届く姿勢を探す
- 直立は腕が伸びきっているので、まず F で手先を下げる
- 真上付近（特異点）で横に動かそうとすると、根元が大きく回るので受け付けない

別のターミナルで距離の表示を出しておくと、ブロックに合わせやすい:

```bash
ros2 run crane_ee_teleop grasp_monitor
```

## テスト

```bash
cd crane_ee_teleop && python3 -m pytest test/
```

（作業中。実行方法・結果は随時追記する）
