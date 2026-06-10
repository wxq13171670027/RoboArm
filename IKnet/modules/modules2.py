from torch import nn
import torch
from torch.distributions import Normal
from torch import distributions as D
from torch.distributions.mixture_same_family import MixtureSameFamily
import torch.nn.functional as F
from modules.sparsemax import *
import matplotlib.pyplot as plt

"""
一层，in_dim -> out_dim
"""
class ProjectionHead(nn.Module):
    def __init__(self, in_dim, out_dim, nlayers = 1):
        #nlayers是隐藏层
        super(ProjectionHead, self).__init__()
        #实现MLP投影头
        self.head = nn.Sequential()
        for i in range(nlayers - 1): 
            """
            实现mlp:
            对每一个i,添加一个名为linear_{i}的线性层和relu_{i}的非线性层,其中线性层的输入输出维度均为in_dim
            """
            self.head.add_module(f"linear_{i}",nn.Linear(in_dim, in_dim))#线性层
            self.head.add_module(f"relu_{i}",nn.ReLU())#激活层
           
        # 实现输出层
        self.head.add_module(f"linear_final", nn.Linear(in_dim, out_dim))

    def forward(self, x):
        return self.head(x)

"""
将feature 投影成weights
"""
class MultiHeadLinearProjection(nn.Module):
    def __init__(self, in_dim, output_size, nlayers=1):
        super(MultiHeadLinearProjection, self).__init__()
        self.linears = nn.ModuleList()
        for i in output_size:
            self.linears.append(ProjectionHead(in_dim, i, nlayers))
        
    def forward(self, features):
        out = []
        for head in self.linears:
            out += [head(features) / (features.shape[1]**0.5)]
        return out

"""
超网络，将位姿提取为feature然后投影成weights
分为两阶段：
1特征提取 2特征投影
1
将输入先拆分为位置和姿态两组，通过不同的MLP进行特征提取，最后将提取的特征合并
然后通过新的全连接层得到目标特征
"""
class HyperNet(nn.Module):
    def __init__(self, cfg):
        super(HyperNet, self).__init__()
        self.num_joints = cfg.num_joints
        self.tempnet_hidden_size = cfg.hypernet_hidden_size // 2
        self.posnet_hidden_layer_sizes = [self.tempnet_hidden_size] * cfg.hypernet_num_hidden_layers
        self.orinet_hidden_layer_sizes = [cfg.hypernet_hidden_size] * cfg.hypernet_num_hidden_layers
        self.hidden_layer_sizes = [cfg.hypernet_hidden_size] * cfg.hypernet_num_hidden_layers

        pos_dims = [3] + self.posnet_hidden_layer_sizes
        ori_dims = [3] + self.orinet_hidden_layer_sizes
        #[3]：输入层维度 self.pos/ori net_hidden_layer_sizes：隐藏层维度列表

        dims = [cfg.embedding_dim * 2] + self.hidden_layer_sizes
        #cfg.embedding_dim * 2：输入层维度，位置和姿态特征融合后的维度 self.hidden_layer_sizes：主网络的隐藏层配置

        self.pos_net = nn.Sequential() #位置网络
        for i in range(len(pos_dims)-1): #根据位置数据维度和隐藏层数量构建位置网络
            self.pos_net.append(nn.Sequential(
                nn.BatchNorm1d(pos_dims[i]),
                nn.Linear(pos_dims[i], pos_dims[i+1]),
                nn.ReLU()
            ))
        self.pos_net.append(nn.Linear(pos_dims[-1], cfg.embedding_dim))
      

        self.ori_net = nn.Sequential() #姿态网络
        for i in range(len(ori_dims)-1): #根据姿态数据维度和隐藏层数量构建位置网络
            self.ori_net.append(nn.Sequential(
                nn.BatchNorm1d(ori_dims[i]),
                nn.Linear(ori_dims[i],ori_dims[i+1]),
                nn.ReLU()
            ))
        self.ori_net.append(nn.Linear(ori_dims[-1], cfg.embedding_dim))

        self.feature_net = nn.Sequential() #特征融合网络，将位置和姿态特征进行融合
        for i in range(len(self.hidden_layer_sizes)-1):
            self.feature_net.append(nn.Sequential(
                nn.BatchNorm1d(self.hidden_layer_sizes[i]),
                nn.Linear(self.hidden_layer_sizes[i], self.hidden_layer_sizes[i+1]),
                nn.ReLU()
            ))
        self.feature_net.append(nn.Linear(self.hidden_layer_sizes[-1], cfg.embedding_dim))

        num_parameters_list = []
        for i in range(1, self.num_joints + 1):
            num_parameters_list += [cfg.jointnet_hidden_size * i,
                                    cfg.jointnet_hidden_size,
                                    cfg.jointnet_hidden_size * cfg.jointnet_output_dim,
                                    cfg.jointnet_output_dim]
        self.projection = MultiHeadLinearProjection(cfg.embedding_dim, num_parameters_list, 1)

    def forward(self, inputs):
        pos_inputs = inputs[:,:3]
        ori_inputs = inputs[:,3:]

        pos_features = self.pos_net(pos_inputs) #位置特征
        ori_features = self.ori_net(ori_inputs) #姿态特征
        #融合特征 拼接位置和姿态特征
        combined_features = torch.cat((pos_features, ori_features), dim=1)
        #通过特征网络进一步处理融合后的特征
        fused_features = self.feature_net(combined_features)
        #生成权重参数
        weights = self.projection(fused_features)

        return weights

"""
利用超网络得到的隐藏层参数计算构建GMM所需参数
"""
class JointNetTemplate(nn.Module):
    def __init__(self, cfg):
        super(JointNetTemplate, self).__init__()
        self.hidden_layer_size = cfg.jointnet_hidden_size
        self.output_dim = cfg.jointnet_output_dim

    def forward(self, input, weights):
        if input.shape[1] == 1:
            out = input * weights[0] + weights[1]
        else:
            out = torch.bmm(input.unsqueeze(1), weights[0].reshape(weights[0].shape[0], input.shape[1], self.hidden_layer_size)).squeeze(1) + weights[1]
        out = torch.relu(out)
        out = torch.bmm(out.unsqueeze(1), weights[2].reshape(weights[2].shape[0], self.hidden_layer_size, self.output_dim)).squeeze(1) + weights[3]
        return out
    
class MainNet(nn.Module):
    def __init__(self, cfg):
        super(MainNet, self).__init__()
        self.num_joints = cfg.num_joints
        self.num_gaussians = cfg.num_gaussians
        self.joint_template = JointNetTemplate(cfg)

    def _create_distribution(self, out):
        """
        创建混合高斯分布
        1. 理解输入输出
        输入参数 out: 神经网络的输出张量，形状为 (batch_size, num_gaussians*3)
        前 num_gaussians个值表示各个高斯分布的均值
        接下来的 num_gaussians个值表示各个高斯分布的对数方差(需要取指数得到方差)
        最后 num_gaussians个值表示混合权重(当 num_gaussians > 1时)
        """
        """
        2. 计算混合权重 selection_weights
        如果 self.num_gaussians == 1:
        创建一个全1的张量作为权重,形状为 (batch_size, 1)
        使用 torch.ones(out.shape[0], 1, device=out.device)
        如果 self.num_gaussians > 1:
        从 out中提取权重部分:out[:, self.num_gaussians*2:]
        使用 Sparsemax()激活函数处理权重(确保权重和为1)
        调用 forward方法:Sparsemax().forward(...)
        """
        # 计算混合权重
        selection_weights = (
            torch.ones(out.shape[0],1,device=out.device) if self.num_gaussians == 1 #out的形状(batch_size,num_gaussians*3)
            else Sparsemax().forward(out[:, self.num_gaussians*2:]) #Sparsemax激活函数归一化，去除部分高斯分布
                                                                    #从num_gaussians*2开始提取权重
        )
        """
        3. 创建混合分布
        使用 D.Categorical创建混合权重分布:
        参数:selection_weights(上一步计算的权重)
        """
        mix = D.Categorical(selection_weights)

        """
        4. 创建高斯分量
        从 out中提取均值部分(参考从 out中提取权重部分:out[:, self.num_gaussians*2:])
        添加一个维度：.unsqueeze(2)使其形状变为 (batch_size, num_gaussians, 1)
        从 out中提取对数方差部分同上
        添加一个维度：.unsqueeze(2)
        取指数得到方差：.exp()
        添加一个小常数避免数值不稳定：+ 1e-7

        使用 D.Normal创建高斯分布:
        参数1:均值张量
        参数2:方差张量

        使用 D.Independent包装高斯分布:
        comp = D.Independent(上一步创建的高斯分布, 1) 这里1表示将最后一个维度视为事件维度
        """
        comp = D.Independent(D.Normal(
            out[:, : self.num_gaussians].unsqueeze(2),
            (out[:, self.num_gaussians:self.num_gaussians*2].unsqueeze(2).exp() + 1e-7),
        ), 1)

        """
        5. 组合分布
        使用 MixtureSameFamily将混合权重分布和高斯分量组合:
        参数1:mix(混合权重分布)
        参数2:comp(高斯分量)
        """
        dist = MixtureSameFamily(mix, comp)
        """
        6. 返回结果
        混合高斯分布
        混合权重
        """
        return dist, selection_weights, out

    def forward(self, x, weights):
        x = x.unsqueeze(2)
        distributions, selection = [], [] #存储高斯分布和选择权重
        for i in range(x.shape[1] - 1):
            #提取对应关节的参数
            out = self.joint_template(x[:, :i + 1].squeeze(2), weights[4 * i : 4 * i + 4])
            #创建高斯分布
            dist, sel, _ = self._create_distribution(out)
            distributions.append(dist)
            selection.append(sel)

        return distributions, selection
    """
    验证函数 采样关节角度
    """
    def validate(self, x, weights, lower, upper, init_joint_angles = None, delta=None):
        
        samples, distributions = [], []
        
        # 如果没有提供delta，创建一个全零的张量
        if delta is None:
            delta = torch.zeros(self.num_joints, device=x.device)

        curr_input = x[:, 0].unsqueeze(1)
        for i in range(self.num_joints):
            out = self.joint_template(curr_input, weights[4*i: 4*i+4])
            dist, _, _ = self._create_distribution(out)

            sample = dist.sample().clip(lower[i], upper[i])

            if init_joint_angles is not None:
                max_attempts=500 #最大采样尝试次数
                epsilon = 0.05 * (upper[i] - lower[i]) #允许的误差范围
                """
                在有初始关节角度的情况下实现拒绝采样
                即：在一定步数内, 如果采样值与初始角度距离过远则直接拒绝, 再次采样
                注意这里每次抽样值都只是第i个关节角度
                """
                for _ in range(max_attempts):
                    #判断采样值是否在允许范围之内
                    is_find = (torch.abs(sample - init_joint_angles[i] - 0.5 * delta[i]) < epsilon)
                    if is_find:
                        # print(i, is_find)
                        break #找到采样值，退出循环
                    else:
                        # print(i, is_find)
                        sample = dist.sample().clip(lower[i], upper[i]) #重新采样

            curr_input = torch.cat((curr_input, sample), dim = 1)
            samples.append(sample)
            distributions.append(dist)

        return samples, distributions

    def validate_seq(self, x, weights, lower, upper, init_joint_angles, delta):
        samples, distributions = [], []
        max_attempts=500
        bias = 0.5 * delta

        curr_input = x[:, 0].unsqueeze(1)
        for i in range(self.num_joints):
            out = self.joint_template(curr_input, weights[4*i:4*i+4])
            dist, _, _ = self._create_distribution(out)

            sample = dist.sample().clip(lower[i], upper[i]) 
            epsilon = 0.05 * (upper[i] - lower[i])

            for _ in range(max_attempts):
                # 当init_joint_angles为None时，不进行拒绝采样，直接接受第一个样本
                if init_joint_angles is None:
                    is_find = True
                else:
                    is_find = (torch.abs(sample - init_joint_angles[i] - bias[i]) < epsilon)
                if is_find:
                    break
                else:
                    sample = dist.sample().clip(lower[i], upper[i])

            curr_input = torch.cat((curr_input, sample), dim = 1)
            samples.append(sample)
            distributions.append(dist)

        return samples, distributions






