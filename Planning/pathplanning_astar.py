import heapq
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from datetime import datetime

class Node:
    """A*算法中的节点类，表示3D空间中的一个点"""
    def __init__(self, position, parent=None):
        self.position = position  # 节点的3D坐标 (x, y, z)
        self.parent = parent      # 父节点，用于回溯路径
        self.g = 0               # 从起点到当前节点的实际代价
        self.h = 0               # 从当前节点到终点的启发式估计代价
        self.f = 0               # 总代价 f = g + h

    def __eq__(self, other):
        """判断两个节点是否相等（位置相同）"""
        return self.position == other.position

    def __lt__(self, other):
        """比较两个节点的f值，用于优先队列排序"""
        return self.f < other.f

    def __hash__(self):
        """使节点可哈希，便于在集合中使用"""
        return hash(self.position)

def heuristic(node, goal):
    """启发式函数：使用曼哈顿距离估计当前节点到终点的代价"""
    return abs(node.position[0] - goal.position[0]) + \
           abs(node.position[1] - goal.position[1]) + \
           abs(node.position[2] - goal.position[2])

def is_obstacle(position, obstacles):
    """检查给定位置是否位于任何障碍物内部"""
    x, y, z = position
    for obstacle in obstacles:
        ox, oy, oz, size_x, size_y, size_z = obstacle
        # 检查位置是否在障碍物边界内（包含1cm的安全距离）
        if (ox - size_x/2 -30 <= x <= ox + size_x/2 +30 and
            oy - size_y/2 -30 <= y <= oy + size_y/2 +30 and
            oz - size_z/2 -30 <= z <= oz + size_z/2 +30):
            return True
    return False

def is_within_bounds(position):
    """检查位置是否在有效的工作空间范围内"""
    # 工作空间是一个球壳：内半径200mm，外半径350mm，中心在(0,0,174)
    return (position[0]**2 + position[1]**2 + (position[2]-174)**2 < 350**2 and
            position[0]**2 + position[1]**2 + (position[2]-174)**2 > 200**2 and
            position[2] > 0)  # z坐标必须大于0

def adjust_goal(original_goal, start_pos, step_size):
    """调整目标点坐标，使其与起点之间的差值能被步长整除"""
    adjusted = []
    for orig, start in zip(original_goal, start_pos):
        delta = orig - start
        steps = round(delta / step_size)  # 计算需要的步数
        adjusted.append(start + steps * step_size)  # 重新计算目标点
    return tuple(adjusted)

def astar(start, goal, obstacles, step, visualize=False):
    """A*算法主函数：在3D空间中寻找从起点到终点的最优路径"""
    
    # 可视化初始化
    if visualize:
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("3D A* Path Planning")
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
    
    # 输入验证
    if not is_within_bounds(start) or not is_within_bounds(goal):
        raise ValueError("起点或终点不在有效边界内")
    if is_obstacle(start, obstacles) or is_obstacle(goal, obstacles):
        raise ValueError("起点或终点位于障碍物内")
    
    # 算法初始化
    open_list = []        # 优先队列，存储待探索节点
    closed_list = set()   # 已探索节点集合
    explored_nodes = []   # 记录所有探索过的节点（用于可视化）
    start_node = Node(start)
    goal_node = Node(goal)
    heapq.heappush(open_list, (start_node.f, start_node))  # 将起点加入开放列表

    # 主循环
    while open_list:
        # 从开放列表中取出f值最小的节点
        current_node = heapq.heappop(open_list)[1]
        explored_nodes.append(current_node.position)

        # 如果到达目标点，回溯构建路径
        if current_node == goal_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1], explored_nodes  # 返回反转的路径（从起点到终点）

        # 将当前节点加入关闭列表
        closed_list.add(current_node.position)

        # 探索当前节点的6个邻居（上下左右前后）
        for dx, dy, dz in [(-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)]:
            # 创建邻居节点
            neighbor = Node((current_node.position[0]+dx*step,
                            current_node.position[1]+dy*step,
                            current_node.position[2]+dz*step), current_node)
            
            # 跳过无效的邻居节点
            if is_obstacle(neighbor.position, obstacles):
                continue  # 障碍物
            if neighbor.position in closed_list:
                continue  # 已探索过
            if not is_within_bounds(neighbor.position):
                continue  # 超出边界

            # TODO: 计算邻居节点的代价
            neighbor.g = current_node.g + step   # 实际代价
            neighbor.h = heuristic(neighbor, goal_node)   # 启发式代价
            neighbor.f = neighbor.g + neighbor.h   # 总代价

            # 如果开放列表中不存在更优的相同节点，则加入开放列表
            if not any(node == neighbor and node.f <= neighbor.f for _, node in open_list):
                heapq.heappush(open_list, (neighbor.f, neighbor))

        # 实时可视化更新（每探索50个节点更新一次）
        if visualize and len(explored_nodes) % 50 == 0:
            ax.cla()
            plot_environment(ax, start, goal, obstacles, explored_nodes)
            plt.pause(0.001)
    
    # 开放列表为空仍未找到路径
    return None, explored_nodes

def plot_obstacles(ax, obstacles):
    """绘制三维障碍物立方体"""
    for obstacle in obstacles:
        ox, oy, oz, sx, sy, sz = obstacle
        # 计算立方体的边界
        x = [ox - sx/2, ox + sx/2]
        y = [oy - sy/2, oy + sy/2]
        z = [oz - sz/2, oz + sz/2]
        
        # 绘制立方体六个面
        for xi in x:
            for yj in y:
                ax.plot([xi, xi], [yj, yj], z, color='gray', alpha=0.3, zorder=5)
        for xi in x:
            for zk in z:
                ax.plot([xi, xi], y, [zk, zk], color='gray', alpha=0.3, zorder=5)
        for yj in y:
            for zk in z:
                ax.plot(x, [yj, yj], [zk, zk], color='gray', alpha=0.3, zorder=5)

def plot_environment(ax, start, goal, obstacles, explored=None, path=None):
    """可视化函数：绘制完整的3D路径规划环境"""
    
    # 绘制探索节点（底层，半透明显示）
    if explored:
        xs, ys, zs = zip(*explored)
        ax.scatter(xs, ys, zs, c='lightgray', alpha=0.15, marker='.', 
                  s=3, zorder=1, label='Explored Nodes')
    
    # 绘制障碍物（中层）
    plot_obstacles(ax, obstacles)
    
    # 绘制路径（上层，蓝色线条）
    if path:
        xs, ys, zs = zip(*path)
        ax.plot(xs, ys, zs, c='dodgerblue', linewidth=3.5, 
               zorder=15, label='Optimal Path')
    
    # 绘制起点终点（顶层，突出显示）
    ax.scatter(*start, c='limegreen', s=120, marker='o', 
              edgecolors='darkgreen', zorder=20, label='Start')
    ax.scatter(*goal, c='red', s=200, marker='*', 
              edgecolors='darkred', zorder=20, label='Goal')
    
    # 设置坐标轴范围（基于探索节点和起终点动态调整）
    all_x = [p[0] for p in explored] if explored else []
    all_y = [p[1] for p in explored] if explored else []
    all_z = [p[2] for p in explored] if explored else []
    ax.set_xlim(min(all_x+[start[0], goal[0]])-50, max(all_x+[start[0], goal[0]])+50)
    ax.set_ylim(min(all_y+[start[1], goal[1]])-50, max(all_y+[start[1], goal[1]])+50)
    ax.set_zlim(min(all_z+[start[2], goal[2]])-50, max(all_z+[start[2], goal[2]])+50)
    
    ax.legend()  # 显示图例

def save_path_to_file(path, filename):
    """将规划好的路径保存到文本文件"""
    with open(filename, 'w') as file:
        file.write("# 轨迹类型: 避障轨迹\n")
        file.write("# 点数: {}\n".format(len(path)))
        file.write("# X(mm) Y(mm) Z(mm)\n")
        for x, y, z in path:
            file.write("{:.2f} {:.2f} {:.2f}\n".format(x, y, z))

# ========== 主程序执行部分 ==========

# A*算法起点终点和障碍物相关参数
start = (120, 270, 57)           # 起点坐标 (x, y, z)
original_goal = (170, -230, 45)   # 终点坐标
obstacles = [                      # 障碍物列表：每个障碍物为(中心x,中心y,中心z,尺寸x,尺寸y,尺寸z)
    (260, 10, 65, 224, 200, 130),
]
step = 10  # A*算法的步长（毫米）

# 调整终点坐标以适应步长要求
adjusted_goal = adjust_goal(original_goal, start, step)
if adjusted_goal != original_goal:
    print(f"提示：终点已从 {original_goal} 调整为 {adjusted_goal} 以满足步长要求")

# 运行A*算法并实时可视化
try:
    path, explored = astar(start, adjusted_goal, obstacles, step, visualize=True)
except ValueError as e:
    print(f"错误：{e}")
    path = None

# 最终结果可视化
final_fig = plt.figure(figsize=(12, 8))
final_ax = final_fig.add_subplot(111, projection='3d')
plot_environment(final_ax, start, adjusted_goal, obstacles, explored, path)
final_ax.view_init(elev=25, azim=45)  # 设置3D视角
plt.show()

# 保存路径文件
if path is not None:
    filename = f"planned_path_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    save_path_to_file(path, filename)
    print(f"路径已保存至：{os.path.abspath(filename)}")
else:
    print('未找到有效路径')