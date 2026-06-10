from pymycobot.mycobot import MyCobot
import time
import sys
from datetime import datetime
import os

def init_robot(port='/dev/ttyAMA0', baudrate=115200):
    """初始化机械臂并检查连接状态"""
    try:
        mc = MyCobot(port=port, baudrate=baudrate)
        mc.power_on()
        # 简单的连接测试
        if mc.is_power_on():
            print("✅ 机械臂连接成功并已上电")
            # 释放所有舵机，使能拖动示教
            mc.release_all_servos()
            print("✅ 已释放所有舵机，可以进行拖动示教")
            return mc
        else:
            print("⚠️ 机械臂未上电，请检查电源")
            return None
    except Exception as e:
        print(f"❌ 连接失败: {str(e)}")
        return None

def format_coordinates(coords):
    """格式化坐标输出"""
    if coords and len(coords) == 6:
        return (f"位置: X={coords[0]:.2f}mm, Y={coords[1]:.2f}mm, Z={coords[2]:.2f}mm\n"
                f"姿态: Rx={coords[3]:.2f}°, Ry={coords[4]:.2f}°, Rz={coords[5]:.2f}°")
    return "无效坐标数据"

def record_coordinates(mc, filename=None, frequency=10, duration=None):
    """记录拖动示教的位姿数据
    
    Args:
        mc: MyCobot对象
        filename: 保存文件名，默认使用时间戳
        frequency: 记录频率(Hz)
        duration: 记录持续时间(秒)，None表示持续记录直到手动停止
    """
    # 如果没有指定文件名，使用时间戳创建文件名
    if filename is None:
        filename = f"cobot_poses_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    
    interval = 1.0 / frequency  # 计算采样间隔
    start_time = time.time()
    count = 0  # 记录数据点数量
    
    print(f"📝 开始记录位姿数据到文件: {filename}")
    print("⚡ 采样频率: {:.1f}Hz (间隔: {:.3f}秒)".format(frequency, interval))
    print("🛑 按 Ctrl+C 停止记录")
    
    try:
        with open(filename, 'w') as f:
            # 写入文件头
            f.write("X,Y,Z,Rx,Ry,Rz\n")
            
            while True:
                current_time = time.time()
                coords = mc.get_coords()
                
                if coords and len(coords) == 6:
                    # 写入坐标数据
                    
                    data_line = f"{coords[0]:.2f} {coords[1]:.2f} {coords[2]:.2f} " \
                               f"{coords[3]:.2f} {coords[4]:.2f} {coords[5]:.2f}\n"
                    f.write(data_line)
                    count += 1
                    
                    # 每10个数据点显示一次状态
                    if count % 10 == 0:
                        print(f"✨ 已记录 {count} 个数据点")
                
                if duration and (current_time - start_time) > duration:
                    break
                
                # 精确控制采样间隔
                sleep_time = interval - (time.time() - current_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print(f"\n📊 记录已停止，共记录 {count} 个数据点")
    except Exception as e:
        print(f"❌ 记录过程出错: {str(e)}")
    finally:
        print(f"💾 数据已保存到: {os.path.abspath(filename)}")

if __name__ == "__main__":
    # 初始化机械臂
    mc = init_robot()
    if mc:
        # 开始记录位姿数据，频率10Hz
        record_coordinates(mc, frequency=10)