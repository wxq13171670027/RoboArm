import pybullet as p
import pybullet_data
import os
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import keyboard  

def calculate_rotation_matrix(angleX, angleY, angleZ):
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(angleX), -math.sin(angleX)],
        [0, math.sin(angleX), math.cos(angleX)]
    ])
    R_y = np.array([
        [math.cos(angleY), 0, math.sin(angleY)],
        [0, 1, 0],
        [-math.sin(angleY), 0, math.cos(angleY)]
    ])
    R_z = np.array([
        [math.cos(angleZ), -math.sin(angleZ), 0],
        [math.sin(angleZ), math.cos(angleZ), 0],
        [0, 0, 1]
    ])
    return R_z @ R_y @ R_x

def pybullet_simulation(queue_imu1, queue_imu2, queue_imu3, queue_imu4, queue_imu5, queue_imu6):
   
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)  
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)  
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)  
 

    p.resetDebugVisualizerCamera(
        cameraDistance=0.8,  
        cameraYaw= -90,        
        cameraPitch=-10,     
        cameraTargetPosition=[0, 0, 0.3] 
    )


    urdf_path = "D:/Robhomework/ai/e2/Experiment2/mycobot_file/new_mycobot_pro_320_pi_2022.urdf" 
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")
    
    robot = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
    

 
    plane_id = p.loadURDF("plane.urdf")

   
    num_joints = p.getNumJoints(robot)

    
    current_joint_positions = [0.0] * num_joints


    imu_init = {
        'imu1': {'R_init_inv': None, 'init_done': False, 'init_request': True},
        'imu2': {'R_init_inv': None, 'init_done': False, 'init_request': True},
        'imu3': {'R_init_inv': None, 'init_done': False, 'init_request': True},
        'imu4': {'R_init_inv': None, 'init_done': False, 'init_request': True},
        'imu5': {'R_init_inv': None, 'init_done': False, 'init_request': True},
        'imu6': {'R_init_inv': None, 'init_done': False, 'init_request': True}
    }


  
    last_print_time = time.time()
    while True:
        if keyboard.is_pressed('space'):
            for key in imu_init:
                imu_init[key]['init_request'] = True

        while not queue_imu1.empty():
            data_imu1 = queue_imu1.get()  
        while not queue_imu2.empty():
            data_imu2 = queue_imu2.get() 
        while not queue_imu3.empty():
            data_imu3 = queue_imu3.get()  
        while not queue_imu4.empty():
            data_imu4 = queue_imu4.get()  
        while not queue_imu5.empty():
            data_imu5 = queue_imu5.get()
        while not queue_imu6.empty():
            data_imu6 = queue_imu6.get()

        
        if 'data_imu1' in locals() and 'data_imu2' in locals() and 'data_imu3' in locals() and 'data_imu4' in locals() and 'data_imu5' in locals() and 'data_imu6' in locals():
           
            for idx, data, key in zip(
                range(1, 7),
                [data_imu1, data_imu2, data_imu3, data_imu4, data_imu5, data_imu6],
                ['imu1', 'imu2', 'imu3', 'imu4', 'imu5', 'imu6']
            ):
                if imu_init[key]['init_request']:
                    angleX, angleY, angleZ = map(math.radians, (data[7], data[8], data[9]))
                    R_init = calculate_rotation_matrix(angleX, angleY, angleZ)
                    
                   
                    if key == 'imu6':
                        R_y_90 = np.array([
                            [math.cos(-math.pi / 2), 0, math.sin(-math.pi / 2)],
                            [0, 1, 0],
                            [-math.sin(-math.pi / 2), 0, math.cos(-math.pi / 2)]
                        ])
                        R_z_90 = np.array([
                            [math.cos(math.pi / 2), -math.sin(math.pi / 2), 0],
                            [math.sin(math.pi / 2), math.cos(math.pi / 2), 0],
                            [0, 0, 1]
                        ])
                        R_init = R_z_90 @ R_y_90 @ R_init
                      

                    imu_init[key]['R_init_inv'] = np.linalg.inv(R_init)
                    imu_init[key]['init_done'] = True
                    imu_init[key]['init_request'] = False
                    print(f"{key} 初始化完成")

           
            def get_relative_angles(data, key):
                angleX, angleY, angleZ = map(math.radians, (data[7], data[8], data[9]))
                
                R_curr = calculate_rotation_matrix(angleX, angleY, angleZ)
               
                if imu_init[key]['R_init_inv'] is not None:

                    R_rel = R_curr @ imu_init[key]['R_init_inv']
                    angles = R.from_matrix(R_rel).as_euler('xyz', degrees=False)
                    return angles[0], angles[1], angles[2]
                else:
                    return angleX, angleY, angleZ
            print
            angleX1, angleY1, angleZ1 = get_relative_angles(data_imu1, 'imu1')
            angleX2, angleY2, angleZ2 = get_relative_angles(data_imu2, 'imu2')
            angleX3, angleY3, angleZ3 = get_relative_angles(data_imu3, 'imu3')
            angleX4, angleY4, angleZ4 = get_relative_angles(data_imu4, 'imu4')
            angleX5, angleY5, angleZ5 = get_relative_angles(data_imu5, 'imu5')
            angleX6, angleY6, angleZ6 = get_relative_angles(data_imu6, 'imu6')

           
            theta1 = angleZ1
            theta2 = math.pi - angleY2
            theta3 = angleZ3
            theta4 = math.pi + angleZ4
            theta5 = -angleY5
            theta6 = angleY6
            # ...需要你填写所需的角度并且处理

            target_angles = [theta1, theta2, theta3 , theta4 , theta5 , theta6 ]

            while len(target_angles) < num_joints:
                target_angles.append(0.0)
            target_angles = target_angles[:num_joints]
            
            for i in range(num_joints):
                if i < len(target_angles):
                    current_joint_positions[i] = target_angles[i]
          
            for j in range(num_joints):
                p.setJointMotorControl2(
                    bodyUniqueId=robot,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=current_joint_positions[j],
                    force=500,
                    maxVelocity=10
                )
            
            link_state = p.getLinkState(robot, num_joints - 1)
            end_effector_position = link_state[4]
            end_effector6_orientation = link_state[5]
          
            now = time.time()
            if now - last_print_time >= 0.1:
                print(f"坐标: [{end_effector_position[0]*1000:.1f},{end_effector_position[1]*1000:.1f},{end_effector_position[2]*1000:.1f}]")
                last_print_time = now
            debug_text = f"EE Pos: [{end_effector_position[0]*1000:.1f},{end_effector_position[1]*1000:.1f},{end_effector_position[2]*1000:.1f}]\nEE Ori: [{end_effector6_orientation[0]:.3f}, {end_effector6_orientation[1]:.3f}, {end_effector6_orientation[2]:.3f}, {end_effector6_orientation[3]:.3f}]"
            if 'text_id' in locals():
                p.removeUserDebugItem(text_id)
            text_id = p.addUserDebugText(debug_text, [0.5, 0, 1], textColorRGB=[1,0,0], textSize=1.5)
        
        
        p.stepSimulation()
        time.sleep(1/240)
        