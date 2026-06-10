from pymycobot.mycobot import MyCobot  # 导入MyCobot机械臂控制库
from pymycobot.genre import Coord      # 导入坐标相关的类定义
import time                            # 导入时间处理模块
import numpy as np                     # 导入numpy用于数学计算

def read_trajectory(file_path):
    """
    读取轨迹文件并解析坐标点信息
    
    Args:
        file_path (str): 轨迹文件的路径
        
    Returns:
        list: 包含轨迹点的列表，每个点为[x, y, z, rx, ry, rz]格式
              其中(x,y,z)为位置坐标，(rx,ry,rz)为姿态角度
    """
    trajectory_points = []
    with open(file_path, 'r') as f:
        # 跳过文件头的前三行（通常包含文件信息或注释）
        for _ in range(1):
            next(f)
        # 逐行读取轨迹点数据
        for line in f:
            # 将每行的字符串分割并转换为浮点数
            x, y, z = map(float, line.strip().split())
            # 添加默认的姿态角度
            rx, ry, rz = 180, 0, 0  # TODO:设置末端执行器的姿态角度（路径跟踪与轨迹规划规划的要求不同）
            trajectory_points.append([x, y, z, rx, ry, rz])
    return trajectory_points

def main():
    # 初始化机械臂，设置串口和波特率
    mc = MyCobot('/dev/ttyAMA0', 115200)  # 使用树莓派的串口进行通信
    mc.power_on()                          # 给机械臂通电

    # #### 开启实时模式
    # mc.set_fresh_mode(1)
    # print("已开启实时控制模式，机械臂将连续执行轨迹指令")
    
    # 等待机械臂启动和初始化
    time.sleep(2)
    '''
    # 将机械臂移动到初始零位姿态
    print("正在回到初始位置...")
    mc.send_angles([0, 0, 0, 0, 0, 0], 10)  # 所有关节角度设为0，速度10度/秒
    time.sleep(10)  # 等待机械臂到达初始位置
    '''
    # 从文件中读取轨迹数据
    trajectory = read_trajectory('planned_path_1.txt')  #TODO:替换成自己的路径
    print("read txt successfully!")
    
    # 设置机械臂运动速度（单位：毫米/秒）
    speed = 10
    
    try:
        # 获取机械臂当前位置信息
        current_coords = mc.get_coords()
        if current_coords:
            print("当前位置:", current_coords)
        else:
            print("无法获取当前位置")
            return
        
        # 移动到轨迹的第一个点（起点）
        start_point = trajectory[0]
        print(f"移动到起点: {start_point}")
        mc.send_coords(start_point, speed, 0)  # 0表示关节运动模式
        time.sleep(10)  # 等待到达起点
        
        # 逐点执行轨迹运动
        for i, point in enumerate(trajectory):
            print(f"执行第 {i+1}/{len(trajectory)} 个轨迹点: {point}")
            
            # 发送坐标指令，控制机械臂运动
            mc.send_coords(point, speed, 0)
            time.sleep(0.5)  # 固定等待时间
            ### 删除固定等待时间，实时模式无需等待
            
            # 注释掉的动态等待时间计算代码
            #if i > 0:
            #    # 计算当前点与上一点之间的欧氏距离
            #    prev_point = trajectory[i-1]
            #    distance = np.sqrt(sum((np.array(point[:3]) - np.array(prev_point[:3]))**2))
            #    # 根据距离和速度计算理论运动时间，并添加0.1秒的缓冲时间
            #    wait_time = (distance / speed) + 0.1
            #    time.sleep(wait_time)
        
        print("轨迹执行完成")
        
    except Exception as e:
        # 捕获并打印执行过程中的任何错误
        print(f"执行过程中出现错误: {e}")

if __name__ == "__main__":
    # 程序入口点，调用main函数
    main() 
