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
repo_id = "crane_plus_grasp_train_trimstill"
root = "/home/ubuntu/dataset/lerobot_dataset/crane_plus_grasp_train_trimstill"
task_name = "grasp"
