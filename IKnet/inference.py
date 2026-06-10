import torch
from modules.modules import *
import ikpy.chain
import numpy as np
import json
import argparse
from scipy.spatial.transform import Rotation as R
from utils.simulator import MyCobotSimulator
from scipy.interpolate import CubicHermiteSpline

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def load_config(path):
    with open(path, 'r') as f:
        config_dict = json.load(f)
    
    # 创建新的argparse命名空间并加载配置
    parser = argparse.ArgumentParser()
    for key, value in config_dict.items():
        parser.add_argument(f'--{key}', type=type(value), default=value)
    
    return parser.parse_args([])  # 传入空列表避免解析命令行参数

def inference(cfg, upper, lower, hypernet, mainnet, sim, init_joint_angles = None, input_positions = None, delta = torch.tensor([[0, 0, 0, 0, 0, 0]], dtype=torch.float).to(device)):
    """
    进行推理计算，找到最小关节移动量对应的解
    :param cfg: 配置参数
    :param upper, lower: 关节角度的上下界
    :param hypernet: 超网络模型
    :param mainnet: 主网络模型
    :param input_positions: 输入位置数组
    :param init_joint_angles: 初始关节角度数组
    :return: 最小关节移动量对应的解
    """
    if input_positions is None:
        # 使用默认测试位置
        input_positions = np.array([[0.2, 0.1, 0.3, 3.14, 0.1, 0.2]])
        print("[WARNING] 使用默认测试位置")

    # 将输入位置转换为torch张量并移动到GPU
    positions = torch.from_numpy(input_positions).float().to(device)
    # 初始化关节角度张量
    joint_angles = torch.tensor([[0, 0, 0, 0, 0, 0]], dtype=torch.float).to(device)

    predicted_weights = hypernet(positions)

    min_rpy_error = float('inf')
    best_solution = None
    # 初始化误差累计变量
    error = 0.0
    count = 0

    sampled_solutions = []
    # 检查init_joint_angles是否为None，避免类型错误
    if init_joint_angles is not None:
        init_joint_angles = init_joint_angles + 0.5 * delta
    for _ in range(cfg.num_solutions_validation * 10):
        sample, _ = mainnet.validate(torch.ones(joint_angles.shape[0], 1).to(device),
                                          predicted_weights, lower, upper, init_joint_angles)
        sampled_solutions.append(sample)

    for sample in sampled_solutions:
        for k in range(len(positions)):
            joint_angle = [sample[i][k].item() for i in range(cfg.num_joints)]

            try:
                if sim.check_collision(joint_angle):
                    print("检测到碰撞，跳过该解")
                    continue

                # 获取目标数据
                target_data = positions[k].detach().cpu().numpy()
                
                # 计算正运动学得到末端位置
                real_frame = sim.forward_kinematics(joint_angle)
                ikpy_end_position = np.array(real_frame[:3, 3]).flatten()
                # 计算姿态
                rotation_matrix = real_frame[:3, :3]
                ikpy_orientation = np.array(R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False))
                
                target_position = target_data[:3]
                target_rpy = target_data[3:]
                # 计算位置误差 (米)
                pos_test_loss = np.sqrt(np.sum((ikpy_end_position - target_position) ** 2))
                # 计算姿态误差 (弧度)
                rpy_test_loss = np.sum(np.abs(ikpy_orientation - target_rpy))
                # 转换角度误差为度数
                rpy_test_loss_deg = np.rad2deg(rpy_test_loss)
                
                # 只输出位置误差小于8mm且角度误差小于12度的解
                if pos_test_loss < 0.008 and rpy_test_loss_deg < 12.0:
                    print(f"关节角度: {np.round(np.rad2deg(joint_angle), 2)}")
                    print(f"末端姿态：{ikpy_orientation}")
                    print(f"目标末端姿态：{target_rpy}")
                    print(f"位置测试损失 (RMSE): {pos_test_loss}")
                    print(f"姿态测试损失 (RMSE)(°): {rpy_test_loss_deg}")
                    print(f"姿态测试损失 (RMSE): {rpy_test_loss}")
                    error += pos_test_loss
                    count += 1
                    # 更新最小误差及其对应的解
                    if rpy_test_loss < min_rpy_error:
                        min_rpy_error = rpy_test_loss
                        best_solution = np.array(joint_angle)

            except Exception as e:
                print(f"正运动学计算出错: {e}")
    # 确保只有在count不为0时才计算平均误差
    if count > 0:
        print(error/count)
    return best_solution, min_rpy_error


if __name__ == '__main__':
    cfg = load_config("/mnt/workspace/runs/exp_1/run_args.json")  #TODO: 自己训练保存的内容
    # 在chain_path前添加绝对路径前缀
    r_arm = ikpy.chain.Chain.from_urdf_file(cfg.chain_path)
        
    upper_bounds = [link.bounds[1] for link in r_arm.links[1:-1]]  # 上界
    lower_bounds = [link.bounds[0] for link in r_arm.links[1:-1]]  # 下界

    upper = torch.tensor(upper_bounds, dtype=torch.float32, device=device)
    lower = torch.tensor(lower_bounds, dtype=torch.float32, device=device)

    hypernet = HyperNet(cfg).to(device)
    mainnet = MainNet(cfg).to(device)

    hypernet.load_state_dict(torch.load("/mnt/workspace/runs/exp_1/best_model.pt")) #TODO: 自己训练保存的内容
    hypernet.eval()

    # 在chain_path前添加绝对路径前缀
    with MyCobotSimulator(cfg.chain_path) as sim:
        solution = inference(cfg, upper, lower, hypernet, mainnet, sim)

