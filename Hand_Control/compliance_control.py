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
m_d = 0.5  # 虚拟质量
b_d = 5.0  # 阻尼
k_d = 100.0 # 刚度
f_e = 1.0 # 外部力
dt = 0.001 # 仿真步长
desired_torque = 10.0  # 目标力矩
torque_control_gain = 0.2  # 力矩控制增益
target_pos = np.deg2rad(90)  # 目标位置

# 积分误差初始化
integral_error = 0.0
integral_limit = 50.0  # 限制积分项大小，防止积分饱和
previous_pos = 0.0  # 记录前一次位置
previous_vel = 0.0  # 记录前一次速度
previous_acc = 0.0  # 记录前一次加速度
in_contact = False
zero_force_count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 获取当前关节位置和速度
        current_pos = data.joint('hinge').qpos[0]
        current_vel = data.joint('hinge').qvel[0]

        derivative_pos = current_pos - previous_pos
        derivative_vel = - current_vel
        current_acc = current_vel - previous_vel
        derivative_acc = - current_acc

        

        # 检测接触状态及读取接触力
        contact_detected = False  # 当前帧是否检测到接触
        measured_force = 0.0      # 当前帧检测到的接触力

        for i in range(data.ncon):
            contact = data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2

            # 修正接触判断条件（确保两个条件不同）
            if (geom1 == 3 and geom2 == 2) or (geom1 == 2 and geom2 == 3):
                contact_detected = True
                efc_start = contact.efc_address
                measured_force = data.efc_force[efc_start]
                break  # 找到一个满足条件的接触即可

        # 状态更新：如果当前检测到接触且力值大于0，则进入接触状态并清零计数器
        if contact_detected and measured_force > 0:
            in_contact = True
            zero_force_count = 0
        else:
            # 如果本帧没有检测到有效接触，但上次处于接触状态，则计数器增加
            if in_contact:
                zero_force_count += 1
                # 当连续3帧没有有效接触时，退出接触状态
                if zero_force_count >= 3:
                    in_contact = False
                    zero_force_count = 0

        # 根据当前状态选择控制策略
        if in_contact:
            print(f"Contact detected! Force: {measured_force:.2f} N")
            # 使用导纳控制调整目标位置
            target_pos = current_pos + ( -measured_force + f_e - b_d * derivative_vel - m_d * derivative_acc)/k_d
        

        # PID 控制部分
        error = target_pos - current_pos
        integral_error += error
        integral_error = np.clip(integral_error, -integral_limit, integral_limit)
        derivative_pos = current_pos - previous_pos
        torque = kp * error + ki * integral_error + kd * derivative_pos

        data.ctrl[0] = torque

        # 更新上一帧数据
        previous_pos = current_pos
        previous_vel = current_vel
        previous_acc = current_vel - previous_vel

        # 步进仿真并更新可视化
        mujoco.mj_step(model, data)
        print(f"Actuator torque: {torque:.2f} Nm")
        print(f"Current position: {np.rad2deg(current_pos):.2f} degrees")
        print(f"Target position: {np.rad2deg(target_pos):.2f} degrees")
        viewer.sync()
        time.sleep(0.001)  # 控制运行速度