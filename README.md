# pai-final-crane-teleop

Physical AI 応用編1 最終課題（Daisuke-Hoshina）。

Gazebo 上の CRANE+ V2 に、キーボードで手先を動かす「手先空間テレオペ」を作り、
既存の関節空間テレオペと比べて、模倣学習（ACT）のデモと方策がどう変わるかを調べる。

ベースにしたサンプル: 「知能ロボットシステムエンジニアリング第5回 ROS 2入門（2/2）」
（CRANE+ のテレオペ → rosbag 記録 → LeRobot 形式へ変換 → ACT 学習 → 推論）。

## 構成

- `crane_ee_teleop/` … ROS 2 パッケージ（ament_python）
  - `crane_ee_teleop/kinematics.py` … CRANE+ V2 の順運動学・逆運動学（解析解。ROS に依存しない）

## テスト

```bash
cd crane_ee_teleop && python3 -m pytest test/
```

（作業中。実行方法・結果は随時追記する）
