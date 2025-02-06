import mujoco
import mujoco.viewer
import numpy as np
import os
import rclpy
from angle_subscriber import AngleSubscriber

# 加载 XML 文件
current_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(current_dir, "hinge.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 初始化 ROS2 节点并订阅角度话题
rclpy.init()
angle_subscriber = AngleSubscriber()

# 打开 MuJoCo Viewer 进行仿真
with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok() and viewer.is_running():
            # 读取 ROS2 中的角度值
            rclpy.spin_once(angle_subscriber)
            angles = angle_subscriber.latest_angles
            print(f"Received angles: {angles}")

            # 更新 MuJoCo 模型中的关节角度
            if angles:
                data.qpos[0] = np.deg2rad(angles[0])  # 设置关节角度（转换为弧度）

            # 进行物理仿真一步
            mujoco.mj_step(model, data)
            viewer.sync()  # 同步渲染视图

    except KeyboardInterrupt:
        pass
    finally:
        angle_subscriber.destroy_node()
        rclpy.shutdown()
