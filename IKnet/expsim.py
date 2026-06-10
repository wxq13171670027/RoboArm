import pybullet as p
import pybullet_data
import os
import time
import numpy as np
import sys

def pybullet_simulation(joint_angles=None):
    # 在本地Windows环境下使用GUI模式运行，显示3D仿真界面
    print("在本地环境使用GUI模式运行仿真，显示机械臂3D界面...")
    try:
        # 尝试连接GUI模式
        p.connect(p.GUI)
        
        # 设置相机视角，以便更好地查看机械臂
        p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.3])
        print("成功启动GUI界面!")
    except Exception as e:
        print(f"GUI模式启动失败: {e}")
        print("尝试使用DIRECT模式(无头模式)...")
        p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 导入arm_files中的URDF文件
    urdf_path = os.path.join("arm_files", "new_mycobot_pro_320_pi_2022.urdf")
    
    # 确保文件路径正确
    if not os.path.exists(urdf_path):
        # 尝试使用绝对路径
        urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_files", "new_mycobot_pro_320_pi_2022.urdf")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at {urdf_path}")
    
    # 加载机械臂URDF
    robot = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
    
    # 加载地面
    plane_id = p.loadURDF("plane.urdf")
    
    # 获取关节数量
    num_joints = p.getNumJoints(robot)
    print(f"机械臂关节数量: {num_joints}")
    
    # 打印关节信息
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        print(f"关节 {i}: {joint_info[1].decode('utf-8')}, 类型: {joint_info[2]}")
    
    # 初始化关节位置
    current_joint_positions = [0.0] * num_joints
    
    # 如果提供了初始关节角度，则使用它
    if joint_angles is not None:
        # 确保关节角度数量不超过机械臂关节数量
        joint_angles = joint_angles[:num_joints]
        for i in range(len(joint_angles)):
            # 检查是否是可移动关节
            joint_info = p.getJointInfo(robot, i)
            if joint_info[2] != p.JOINT_FIXED:
                current_joint_positions[i] = joint_angles[i]
    
    # 设置初始关节角度
    for j in range(num_joints):
        joint_info = p.getJointInfo(robot, j)
        if joint_info[2] != p.JOINT_FIXED:  # 只控制可移动关节
            p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=current_joint_positions[j],
                force=500,
                maxVelocity=10
            )
    
    print("\n机械臂仿真已启动!")
    print("使用键盘输入新的关节角度 (格式: j1 j2 j3 j4 j5 j6，单位: 弧度)")
    print("或者输入 'q' 退出仿真")
    
    # 主循环
    try:
        # 设置初始关节角度并运行仿真
        print("当前关节角度 (弧度):", [current_joint_positions[j] for j in range(num_joints) if p.getJointInfo(robot, j)[2] != p.JOINT_FIXED])
        print("当前关节角度 (度):", [np.rad2deg(current_joint_positions[j]) for j in range(num_joints) if p.getJointInfo(robot, j)[2] != p.JOINT_FIXED])
        
        # 运行单步仿真
        p.stepSimulation()
        
        # 获取并打印末端执行器位置和姿态
        link_state = p.getLinkState(robot, num_joints - 1)
        end_effector_position = link_state[4]  # 世界坐标系中的位置
        end_effector_orientation = link_state[5]  # 四元数
        
        # 转换四元数为欧拉角（RPY，单位：度）
        rpy_orientation = np.rad2deg(p.getEulerFromQuaternion(end_effector_orientation))
        
        print(f"末端执行器位置 (mm): [{end_effector_position[0]*1000:.1f}, {end_effector_position[1]*1000:.1f}, {end_effector_position[2]*1000:.1f}]")
        print(f"末端执行器姿态 (度): [{rpy_orientation[0]:.1f}, {rpy_orientation[1]:.1f}, {rpy_orientation[2]:.1f}]")
        
        # 命令行交互更新关节角度
        while True:
            user_input = input("\n请输入新的关节角度 (用空格分隔，单位: 弧度)，或输入'q'退出: ")
            
            if user_input.lower() == 'q':
                print("退出仿真...")
                break
            
            try:
                # 解析输入的关节角度
                new_angles = list(map(float, user_input.split()))
                
                # 确保输入的角度数量不超过机械臂关节数量
                new_angles = new_angles[:num_joints]
                
                # 更新关节角度
                for j in range(len(new_angles)):
                    joint_info = p.getJointInfo(robot, j)
                    if joint_info[2] != p.JOINT_FIXED:  # 只控制可移动关节
                        current_joint_positions[j] = new_angles[j]
                        p.setJointMotorControl2(
                            bodyUniqueId=robot,
                            jointIndex=j,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=current_joint_positions[j],
                            force=500,
                            maxVelocity=10
                        )
                
                # 运行多步仿真应用新的关节角度，确保变化生效
                for _ in range(100):  # 运行100步仿真
                    p.stepSimulation()
                    time.sleep(0.01)  # 短暂延迟，让GUI能跟上更新
                
                # 获取并打印更新后的关节角度
                active_joint_angles_rad = [current_joint_positions[j] for j in range(num_joints) if p.getJointInfo(robot, j)[2] != p.JOINT_FIXED]
                active_joint_angles_deg = [np.rad2deg(angle) for angle in active_joint_angles_rad]
                print(f"已更新关节角度 (弧度): {active_joint_angles_rad}")
                print(f"已更新关节角度 (度): {active_joint_angles_deg}")
                
                # 获取并打印更新后的末端执行器位置和姿态
                link_state = p.getLinkState(robot, num_joints - 1)
                end_effector_position = link_state[4]  # 世界坐标系中的位置
                end_effector_orientation = link_state[5]  # 四元数
                
                # 转换四元数为欧拉角（RPY，单位：度）
                rpy_orientation = np.rad2deg(p.getEulerFromQuaternion(end_effector_orientation))
                
                print(f"末端执行器位置 (mm): [{end_effector_position[0]*1000:.1f}, {end_effector_position[1]*1000:.1f}, {end_effector_position[2]*1000:.1f}]")
                print(f"末端执行器姿态 (度): [{rpy_orientation[0]:.1f}, {rpy_orientation[1]:.1f}, {rpy_orientation[2]:.1f}]")
                
            except ValueError:
                print("无效的输入，请输入数字，用空格分隔")
    
    except KeyboardInterrupt:
        print("\n仿真被中断")
    finally:
        # 断开连接
        p.disconnect()

def main():
    # 检查命令行参数
    joint_angles = None
    if len(sys.argv) > 1:
        try:
            # 从命令行参数获取关节角度
            joint_angles = list(map(float, sys.argv[1:7]))
            print(f"从命令行参数设置初始关节角度: {joint_angles}")
        except ValueError:
            print("命令行参数格式错误，使用默认关节角度")
    
    # 启动仿真
    pybullet_simulation(joint_angles)

if __name__ == "__main__":
    main()