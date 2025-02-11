import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# 初始化模型和仿真
current_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(current_dir, "finger.xml")
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 控制参数
kp = 50.0  # 比例增益
ki = 50.0  # 积分增益
kd = 5.0   # 增大阻尼，增强柔顺控制特性
desired_torque = 10.0  # 目标力矩
torque_control_gain = 0.2  # 力矩控制增益
target_pos = np.deg2rad(90)  # 目标位置
adaptive_target_pos = target_pos  # 用于适应外力的动态目标位置
compliance_gain = 0.05  # 柔顺增益，控制目标位置变化速率

# 积分误差初始化
integral_error = 0.0
integral_limit = 50.0  # 限制积分项大小，防止积分饱和

# visualize contact frames and forces, make body transparent
options = mujoco.MjvOption()
mujoco.mjv_defaultOption(options)
options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

# 创建查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 获取当前关节位置和速度
        current_pos = data.joint('hinge').qpos[0]
        current_vel = data.joint('hinge').qvel[0]

        # **计算当前力矩误差**
        current_actuator_torque = data.qfrc_actuator[0]
        torque_error = desired_torque - current_actuator_torque

        # **计算环境交互力**
        external_torque = np.abs(current_actuator_torque - torque_error)  # 估算外力
        
        # **基于外力调整目标位置**
        if external_torque > 20.0:  # 设定一个阈值，例如 5 Nm 以上认为是接触障碍物
            adaptive_target_pos -= compliance_gain * (external_torque - 5.0)
            adaptive_target_pos = max(np.deg2rad(45), adaptive_target_pos)  # 限制最小角度，防止无限后退
        else:
            # **恢复原目标位置**
            adaptive_target_pos += compliance_gain * (target_pos - adaptive_target_pos)

        # **计算误差**
        error = adaptive_target_pos - current_pos

        # **积分误差更新**
        integral_error += error * 0.001  # 积分项（0.001 代表仿真时间步长）

        # **积分限幅**
        integral_error = np.clip(integral_error, -integral_limit, integral_limit)

        # **计算 PID 控制输出**
        torque = kp * error + ki * integral_error - kd * current_vel

        # **计算力控修正项**
        torque_correction = torque_control_gain * torque_error

        # **添加力控修正项**
        torque += torque_correction

        # **应用力矩**
        data.ctrl[0] = torque
        
        # 步进仿真
        mujoco.mj_step(model, data)

        # 记录数据
        print(f"Actuator torque: {torque:.2f} Nm")
        print(f"External torque: {external_torque:.2f} Nm")
        print(f"Current position: {np.rad2deg(current_pos):.2f} degrees")
        print(f"Adaptive target position: {np.rad2deg(adaptive_target_pos):.2f} degrees")

        # 更新可视化
        viewer.sync()
        time.sleep(0.01)  # 控制运行速度
