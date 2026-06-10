import numpy as np
import pybullet as p
import pybullet_data

def DH_table(alpha, a, theta, d):
    """根据改进DH参数计算变换矩阵"""
    return np.matrix([
        [np.cos(theta),-np.sin(theta),0,a],
        [np.sin(theta)*np.cos(alpha),np.cos(theta)*np.cos(alpha),-np.sin(alpha),-np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha),np.cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d],
        [0,0,0,1]
    ])


class MyCobotSimulator:
    def __init__(self, urdf_path, num_joints = 6):
        self.urdf_path = urdf_path
        self.num_joints = num_joints
        self.client = None
        self.robot_id = None
        self.plane_id = None

    def __enter__(self):
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(
            self.urdf_path,
            useFixedBase = True,
            flags = p.URDF_USE_SELF_COLLISION
        )
        self.plane_id = p.loadURDF("plane.urdf")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        p.disconnect(self.client)

    def forward_kinematics(self, joint_angles): 
        """正向运动学：返回末端变换矩阵"""
        temp = np.pi / 2
        T1 = DH_table(0, 0, joint_angles[0], 0.173)
        T2 = DH_table(temp, 0, joint_angles[1] - temp, 0)
        T3 = DH_table(0, -0.13635, joint_angles[2], 0)
        T4 = DH_table(0, -0.1195, joint_angles[3] - temp, 0.082)
        T5 = DH_table(temp, 0, joint_angles[4], 0.09415)
        T6 = DH_table(-temp, 0, joint_angles[5], 0.06635)
        self.fk = T1@T2@T3@T4@T5@T6
        return self.fk
    
    def get_pos(self):
        """
        获取末端执行器的位置
        """
        return self.fk[:3, 3].A1
    
    def get_rpy(self):
        """
        获取末端执行器的欧拉角 (roll, pitch, yaw)
        """
        rotation = self.fk[:3, :3]
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        pitch = np.arctan2(-rotation[2, 0], np.sqrt(rotation[2, 1]**2 + rotation[2, 2]**2))
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        return np.array([roll, pitch, yaw])


    def check_collision(self, joint_angles):
        """检查自碰撞和地面碰撞"""
        for i in range(self.num_joints):
            p.resetJointState(self.robot_id, i, joint_angles[i])

        # 执行仿真以更新碰撞检测
        p.stepSimulation()

        # 自碰撞
        for i in range(self.num_joints):
            for j in range(i + 2, self.num_joints):
                if p.getContactPoints(self.robot_id, self.robot_id, i, j):
                    return True

        # 地面碰撞
        if p.getContactPoints(self.robot_id, self.plane_id):
            return True

        return False