from torch.utils.data import DataLoader
from torch.amp import GradScaler, autocast
import torch
from modules.modules import *
from utils.dataset import IKDataset, IKDatasetVal
import ikpy.chain
from scipy.spatial.transform import Rotation as R
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import json

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def read_normalization_data(): # 读取归一化数据
    with open("data/normalization.json",'r') as f:   #这里地址由data_generator函数保存
        normalization_data = json.load(f)
    pos_min = np.array(normalization_data["pos_min"])
    pos_max = np.array(normalization_data["pos_max"])
    rpy_min = np.array(normalization_data["rpy_min"])
    rpy_max = np.array(normalization_data["rpy_max"])
    return pos_min, pos_max, rpy_min, rpy_max

def train(cfg):
    r_arm = ikpy.chain.Chain.from_urdf_file(cfg.chain_path)

    # 利用urdf文件获得关节上下界
    upper_bounds = [link.bounds[1] for link in r_arm.links[1:-1]]  # 上界
    lower_bounds = [link.bounds[0] for link in r_arm.links[1:-1]]  # 下界

    upper = torch.tensor(upper_bounds, dtype=torch.float32, device=device)
    lower = torch.tensor(lower_bounds, dtype=torch.float32, device=device)
    
    # 加载数据
    train_dataset = IKDataset(cfg.train_data_path)
    test_dataset = IKDatasetVal(cfg.test_data_path)

    train_dataloader = DataLoader(train_dataset, batch_size = cfg.batch_size, 
                                  shuffle = True, num_workers=0, pin_memory = True)
    test_dataloader = DataLoader(test_dataset, batch_size = cfg.batch_size, 
                                 shuffle = False, num_workers = 0, pin_memory = True)
    
    hypernet = HyperNet(cfg).to(device)
    mainnet = MainNet(cfg).to(device)
    optimizer = torch.optim.Adam(hypernet.parameters(), lr = cfg.lr)
    scaler = GradScaler(device=device.type)
    
    # loss数据初始化
    pos_min, pos_max, rpy_min, rpy_max = read_normalization_data()
    pos_norm = pos_max - pos_min
    rpy_norm = rpy_max - rpy_min

    train_counter, test_counter = 0, 0
    train_loss, test_loss = 0.0, 0.0
    pos_test_loss,  ori_test_loss = 0.0, 0.0

    best_test_loss = np.inf
    epochs_without_improvements = 0

    train_losses = []
    pos_losses = []
    ori_losses = []

    for epoch in range(cfg.num_epochs):
        hypernet.train()
        for positions, joint_angles in train_dataloader:
            positions = positions.to(device)
            joint_angles = joint_angles.to(device)
            output = torch.cat((torch.ones(joint_angles.shape[0], 1).to(device), joint_angles), dim = 1)

            optimizer.zero_grad()
            with autocast(device_type=device.type, dtype=torch.float16):
                predicted_weights = hypernet(positions)
                distributions, _ = mainnet(output, predicted_weights)
                losses = [-torch.mean(distributions[i].log_prob(joint_angles[:, i].unsqueeze(1))) 
                          for i in range(len(distributions))]
                # 负对数似然函数，给出已有分布，算对应点的概率密度的负对数
                loss = sum(losses)/len(losses)
                
            scaler.scale(loss).backward()
            if cfg.grad_clip > 0:
                scaler.unscale_(optimizer)  # 取消缩放以进行梯度裁剪
                torch.nn.utils.clip_grad_norm_(hypernet.parameters(), cfg.grad_clip)
            scaler.step(optimizer)
            scaler.update()

            train_counter += 1
            train_loss += loss.item()
            
        train_losses.append(train_loss / train_counter)
        print(f"Train loss (Likelihood) {train_losses[-1]}")
        train_loss, train_counter = 0, 0
        delta = torch.zeros(6, device=device)

        if epoch % 2 == 0:
            sampled = []
            hypernet.eval()
            for positions, joint_angles in test_dataloader:
                positions = positions.to(device)
                joint_angles = joint_angles.to(device)
                predicted_weights = hypernet(positions)

                for _ in range(cfg.num_solutions_validation):
                    # 使用validate_seq方法并正确传递参数
                    sample, distributions = mainnet.validate_seq(torch.ones(joint_angles.shape[0], 1).to(device),
                                                                   predicted_weights, lower, upper, None, delta)
                    sampled.append(sample)

            for sample in sampled:
                for k in range(len(positions)):
                    joint_angles =[0] + [sample[i][k].item() for i in range(cfg.num_joints)] + [0]
                    
                    real_frame = r_arm.forward_kinematics(joint_angles) #计算正运动学得到末端位置和姿态
                    end_position = np.array(real_frame[:3, 3]) #提取末端位置
                    rotation_matrix = real_frame[:3, :3] #末端姿态

                    #转换为X-Y-Z欧拉角
                    end_orientation = np.array(R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False))
                    #获取目标数据
                    target_data = positions[k].detach().cpu().numpy()
                    target_position = target_data[:3]
                    target_orientation = target_data[3:]

                    # 计算归一化前的位置均方根误差（RMSE）
                    pos_test_loss += np.sqrt(np.sum((end_position - target_position) ** 2))

                    # 计算归一化前的姿态均方根误差（RMSE）
                    ori_test_loss += np.sqrt(np.sum((end_orientation - target_orientation) ** 2))
                    
                    end_position = end_position / pos_norm
                    end_orientation = end_orientation / rpy_norm
                    target_position = target_position / pos_norm
                    target_orientation = target_orientation / rpy_norm

                    # 计算归一化后的位置均方根误差（RMSE）
                    position_error = np.sqrt(np.sum((end_position - target_position) ** 2))
                    # 计算归一化后的姿态均方根误差（RMSE）
                    orientation_error = np.sqrt(np.sum((end_orientation - target_orientation) ** 2))
                    
                    # 计算总误差（可按需调整权重等）
                    test_loss += position_error + orientation_error
                    test_counter += 1
                
            final_test_loss = test_loss / test_counter
            pos_losses.append(pos_test_loss / test_counter)
            ori_losses.append(ori_test_loss / test_counter)
            print(f"Position test loss (RMSE){pos_losses[-1]}")
            print(f"Orientation test loss (RMSE){ori_losses[-1]}")
            print()

            test_loss, test_counter = 0, 0
            pos_test_loss, ori_test_loss = 0, 0

            plt.plot(range(len(train_losses)), train_losses, label = 'train')
            plt.savefig(f'{cfg.exp_dir}/train_plot.png')
            plt.clf()
            plt.plot(range(len(pos_losses)), pos_losses, label = 'pos_test')
            plt.savefig(f'{cfg.exp_dir}/pos_test_plot.png')
            plt.clf()
            plt.plot(range(len(ori_losses )), ori_losses, label = 'rpy_test')
            plt.savefig(f'{cfg.exp_dir}/rpy_test_plot.png')
            plt.clf()

            torch.save(hypernet.state_dict(), f'{cfg.exp_dir}/last_model.pt')
            torch.save(optimizer.state_dict(), f'{cfg.exp_dir}/last_optimizer.pt')

            if final_test_loss < best_test_loss:
                #实现模型更新和早停止(best_test_loss, epochs_without_improvements)
                epochs_without_improvements = 0
                best_test_loss = final_test_loss

                torch.save(hypernet.state_dict(), f'{cfg.exp_dir}/best_model.pt')
                torch.save(optimizer.state_dict(), f'{cfg.exp_dir}/best_optimizer.pt')
                with open(f'{cfg.exp_dir}/best_test_loss.txt', 'a+') as f:
                    f.write(f'Epoch {epoch} - pos test loss: {pos_losses[-1]}, rpy test loss: {ori_losses[-1]} \n')
            else:
                #实现训练早停止，早停止参数为cfg.early_stopping_epochs
                epochs_without_improvements += 2 #每两轮测试一次，所以+2
                if epochs_without_improvements >= cfg.early_stopping_epochs: #超过一定轮数没有提升
                    print(f"Early stopping at epoch {epoch}")
                    break

def create_exp_dir(cfg):
    if not os.path.exists(cfg.exp_dir):
        os.mkdir(cfg.exp_dir)
    existing_dirs = os.listdir(cfg.exp_dir)
    if existing_dirs:
        sorted_dirs = sorted(existing_dirs, key=lambda x : int(x.split('_')[1]))
        last_exp_num = int(sorted_dirs[-1].split('_')[1])
        exp_name = f"{cfg.exp_dir}/exp_{last_exp_num + 1}"
    else:
        exp_name = f"{cfg.exp_dir}/exp_0"
    os.makedirs(exp_name)
    with open(f'{exp_name}/run_args.json', 'w+') as f:
        json.dump(cfg.__dict__, f, indent=2)
    return exp_name

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--chain-path', type=str, default="arm_files/new_mycobot_pro_320_pi_2022.urdf", help='urdf chain path')     #TODO
    parser.add_argument('--train-data-path', type=str, default="data/train_2500000.hdf5", help='urdf chain path')       #TODO
    parser.add_argument('--test-data-path', type=str, default='data/test_5000.hdf5', help='urdf chain path')           #TODO
    parser.add_argument('--num-joints', type=int, default=6, help='number of joints of the kinematic chain')                #TODO
    
    #以下为可以修改优化的超参数
    parser.add_argument('--early-stopping-epochs', type=int, default=30, help='number of epochs without improvement to trigger end of training') 
    parser.add_argument('--hypernet-input-dim', type=int, default=6, help='number of input to the hypernetwork (f)')
    parser.add_argument('--lr', type=float, default=0.001, help='learning rate')
    parser.add_argument('--num-epochs', type=int, default=10000, help='learning rate')
    parser.add_argument('--num-solutions-validation', type=int, default=5, help='solutions number')
    parser.add_argument('--batch-size', type=int, default=1024, help='batch size')
    parser.add_argument('--embedding-dim', type=int, default=256, help='embedding dimension')
    parser.add_argument('--hypernet-hidden-size', type=int, default=1024, help='hypernetwork (f) number of neurons in hidden layer')
    parser.add_argument('--hypernet-num-hidden-layers', type=int, default=2, help='hypernetwork  (f) number of hidden layers')
    parser.add_argument('--jointnet-hidden-size', type=int, default=256, help='jointnet (g) number of neurons in hidden layer')
    parser.add_argument('--num-gaussians', type=int, default=60, help='number of gaussians for mixture . default=1 no mixture')
    
    parser.add_argument('--with-orientation', action='store_true', default=True, help='Whether to include orientation information')
    parser.add_argument('--grad-clip', type=int, default=1, help='clip norm of gradient')
    parser.add_argument('--exp_dir', type=str, default='runs', help='folder path name to save the experiment')

    parser.set_defaults()
    cfg = parser.parse_args()

    cfg.jointnet_output_dim = cfg.num_gaussians * 2 + cfg.num_gaussians if cfg.num_gaussians != 1 else 2

    full_exp_dir = create_exp_dir(cfg) #这里已经将cfg文件保存，用于后续直接调用

    cfg.exp_dir = full_exp_dir

    train(cfg)
