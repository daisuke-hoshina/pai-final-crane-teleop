# 出典: https://github.com/matsuolab/pai_training（commit b4d6b54、Physical AI 応用編1 の演習用リポジトリ）の configs/crane_features.py を写して変えたもの。
features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
    "observation.environment_state": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
    "action": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
}
# Below is used for the dataset conversion
robot_type = "crane_plus"
# repo_id と root は前の実験（G6）の名前のまま実行した。データセットの保存先は conversion_config.yaml の output_dir で決まり、
# repo_id はデータセットのメタ情報に入る名前だけに使われる（学習は train_config.yaml の repo_id・dataset_root で読む）
repo_id = "crane_plus_grasp_train_trimstill"
root = "/home/ubuntu/dataset/lerobot_dataset/crane_plus_grasp_train_trimstill"
task_name = "grasp"
