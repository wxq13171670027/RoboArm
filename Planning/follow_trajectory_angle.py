from pymycobot.mycobot import MyCobot  # 导入MyCobot机械臂控制库
import time                            # 导入时间处理模块

def read_angles(file_path):
    """
    读取关节角文件并解析角度信息
    Args:
        file_path (str): 关节角文件的路径
    Returns:
        list: 包含关节角的列表，每个点为[j1, j2, j3, j4, j5, j6]格式
              其中j1-j6为六个关节的角度
    """
    angle_points = []
    with open(file_path, 'r') as f:
        # 跳过文件头的第一行（标题行）
        next(f)
        # 逐行读取关节角数据
        for line in f:
            # 将每行的字符串分割并转换为浮点数
            angles = list(map(float, line.strip().split()))
            if len(angles) == 6:
                angle_points.append(angles)
    return angle_points

def main():
    # 初始化机械臂，设置串口和波特率
    mc = MyCobot('/dev/ttyAMA0', 115200)  # 使用树莓派的串口进行通信
    mc.power_on()                          # 给机械臂通电
    
    # 等待机械臂启动和初始化
    time.sleep(2)
    
    # 从文件中读取关节角数据
    angles_list = read_angles('ik_30.txt')  #路径
    print(f"成功读取 {len(angles_list)} 个关节角点")
    
    # 设置机械臂运动速度（单位：度/秒）
    speed = 10
    
    try:
        # 获取机械臂当前关节角度
        current_angles = mc.get_angles()
        if current_angles:
            print("当前关节角度:", current_angles)
        else:
            print("无法获取当前关节角度")
            return
        
        # 移动到第一个关节角点（起点）
        start_angles = angles_list[0]
        print(f"移动到起点关节角: {start_angles}")
        mc.send_angles(start_angles, speed)
        time.sleep(10)  # 等待到达起点
        
        # 逐个执行关节角运动
        for i, angles in enumerate(angles_list):
            print(f"执行第 {i+1}/{len(angles_list)} 个关节角点: {angles}")
            
            # 发送关节角指令，控制机械臂运动
            mc.send_angles(angles, speed)
            time.sleep(0.5)  # 固定等待时间
        
        print("关节角轨迹执行完成")
        
    except Exception as e:
        # 捕获并打印执行过程中的任何错误
        print(f"执行过程中出现错误: {e}")

if __name__ == "__main__":
    # 程序入口点，调用main函数
    main() 