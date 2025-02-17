import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys

# 解析命令行参数，选择 XML 模型文件
if len(sys.argv) > 1:
    model_file = sys.argv[1]  # 用户提供的文件名
else:
    model_file = "finger.xml"  # 默认文件

# 初始化模型和仿真
current_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(current_dir, model_file)

if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"模型文件 {MODEL_PATH} 不存在，请检查路径！")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 控制参数
kp = 14.0    # 比例增益
ki = 0.0     # 积分增益
kd = 0.1    # 微分增益
# 力控制增益
ad = 10       # 0阶导微分增益
bd = 0.0   # 一阶导微分增益
md = 0.0     # 二阶导微分增益
desired_torque = 10.0  # 目标力矩
torque_control_gain = 0.2  # 力矩控制增益
target_pos = np.deg2rad(90)  # 目标位置

# 积分误差初始化
integral_error = 0.0
integral_limit = 50.0  # 限制积分项大小，防止积分饱和
previous_pos = 0.0  # 记录前一次位置
previous_vel = 0.0  # 记录前一次速度
previous_acc = 0.0  # 记录前一次加速度

# 创建查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():

        # 读取传感器数据
        # print(f"Indicator contact force:{data.sensordata[0]:.2f}, {data.sensordata[1]:.2f} N")

        # print(f"touch force: {data.sensordata[3]:.2f}")

        # 获取当前关节位置和速度
        current_pos = data.joint('hinge').qpos[0]
        current_vel = data.joint('hinge').qvel[0]

        

        # **应用力矩**
        data.ctrl[0] = 100

        # 获取接触数据
        for i in range(data.ncon):
            contact = data.contact[i]

            # 获取接触的 geom 索引
            geom1 = contact.geom1
            geom2 = contact.geom2

            print(f"Geom1: {geom1}, Geom2: {geom2}")

            indicator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "indicator_geom")
            box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box")

            print(f"Indicator ID: {indicator_id}, Box ID: {box_id}")


            # 打印接触的信息（位置和力）
            if (geom1 == indicator_id and geom2 == box_id) or (geom1 == box_id and geom2 == indicator_id):
                print(f"########################3")
                print(f"Contact position {contact.pos}")

        # 步进仿真
        mujoco.mj_step(model, data)

        # 记录数据
        # print(f"Actuator torque: {torque:.2f} Nm")
        # print(f"Current position: {np.rad2deg(current_pos):.2f} degrees")
        # print(f"Adaptive target position: {np.rad2deg(target_pos):.2f} degrees")

        # 更新可视化
        viewer.sync()
        time.sleep(0.01)  # 控制运行速度

        
                
