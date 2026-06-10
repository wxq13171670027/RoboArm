import h5py
from torch.utils.data import Dataset, DataLoader
import torch


class IKDataset(Dataset):
    def __init__(self, file_path):
        self.data = h5py.File(file_path, 'r')
        self.results = self.data['results']  # 缓存结果数据集
        self.inputs = self.data['inputs']    # 缓存输入数据集

    def __len__(self):
        return len(self.results)

    def __getitem__(self, idx):
        positions = torch.Tensor(self.results[idx])
        joint_angles = torch.Tensor(self.inputs[idx])
        input_tensor = positions.squeeze(0)
        
        return input_tensor, joint_angles

class IKDatasetVal(Dataset):
    def __init__(self, file_path):
        self.data = h5py.File(file_path, 'r')

    def __len__(self):
        return len(self.data.get('results'))

    def __getitem__(self, idx):
        positions = torch.Tensor(self.data.get('results')[len(self.data.get('results')) - idx - 1])
        joint_angles = torch.Tensor(self.data.get('inputs')[len(self.data.get('results')) - idx - 1])
        input = positions.squeeze(0)
        return input, joint_angles


 


