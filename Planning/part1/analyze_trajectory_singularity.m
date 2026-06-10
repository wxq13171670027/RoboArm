%% 分析特定轨迹上的奇异点
clear;
clc;

%% 创建UR5机器人模型
ur5 = loadrobot('universalUR5', 'DataFormat', 'row');  % 直接加载UR5模型

%% 定义示例轨迹（圆形轨迹）
% 圆形轨迹参数
center = [0.4, 0, 0.2];  % 圆心位置
radius = 0.15;           % 半径
num_points = 100;        % 采样点数

% 生成圆形轨迹点
theta = linspace(0, 2*pi, num_points);
trajectory_points = zeros(num_points, 3);
for i = 1:num_points
    trajectory_points(i,:) = center + radius * [cos(theta(i)), sin(theta(i)), 0];
end

%% 计算轨迹上每个点的逆解和奇异性
singular_threshold = 1e-3;
condition_numbers = zeros(num_points, 1);
singular_points = [];
joint_configs = cell(num_points, 1);

% 创建IK求解器
ik = inverseKinematics('RigidBodyTree', ur5);
weights = [1 1 1 1 1 1];  % 权重向量

% 设置初始构型
initial_config = homeConfiguration(ur5);
initial_joints = [0, -pi/6, -pi/2, -pi/3, -pi/2, 0];  % 初始关节角度
for i = 1:6
    initial_config(i) = initial_joints(i);
end

fprintf('开始分析轨迹...\n');
fprintf('总计 %d 个轨迹点\n', num_points);

% 分析每个轨迹点
for i = 1:num_points
    try
        % 创建目标位姿
        tform = trvec2tform(trajectory_points(i,:)) * eul2tform([theta(i)/2, 0, pi/2]);
        
        % 打印调试信息
        fprintf('处理轨迹点 %d:\n', i);
        fprintf('位置: [%.3f, %.3f, %.3f]\n', trajectory_points(i,:));
        
        % 计算逆解
        if i == 1
            [config, info] = ik('tool0', tform, weights, initial_config);
        else
            [config, info] = ik('tool0', tform, weights, joint_configs{i-1});
        end
        
        % 存储配置
        joint_configs{i} = config;
        
        % 验证解的正确性
        fk = getTransform(ur5, config, 'tool0');
        pos_error = norm(tform2trvec(fk) - trajectory_points(i,:));
        if pos_error > 0.01
            fprintf('警告：位置误差较大 (%.3f m)\n', pos_error);
        else
            fprintf('成功：位置误差在允许范围内 (%.3f m)\n', pos_error);
        end
        
        % 计算雅可比矩阵和奇异性
        J = geometricJacobian(ur5, config, 'tool0');
        s = svd(J);
        condition_numbers(i) = max(s)/min(s);
        
        if min(s) < singular_threshold
            singular_points = [singular_points; i];
            fprintf('在轨迹点 %d 处检测到奇异点\n', i);
            fprintf('条件数: %f\n', condition_numbers(i));
        end
        
    catch e
        fprintf('在轨迹点 %d 处无法求解逆运动学:\n', i);
        fprintf('错误信息: %s\n', e.message);
        if i > 1
            joint_configs{i} = joint_configs{i-1};
        else
            joint_configs{i} = initial_config;
        end
    end
end

%% 可视化结果
figure('Name', '轨迹和奇异点分析');

% 绘制轨迹
subplot(2,2,1);
plot3(trajectory_points(:,1), trajectory_points(:,2), trajectory_points(:,3), 'b-');
hold on;
if ~isempty(singular_points)
    plot3(trajectory_points(singular_points,1), ...
          trajectory_points(singular_points,2), ...
          trajectory_points(singular_points,3), 'ro', 'MarkerSize', 10);
end
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('轨迹和奇异点位置');

% 绘制条件数
subplot(2,2,2);
plot(1:num_points, condition_numbers, 'b-');
hold on;
if ~isempty(singular_points)
    plot(singular_points, condition_numbers(singular_points), 'ro', 'MarkerSize', 10);
end
xlabel('轨迹点序号');
ylabel('条件数');
title('轨迹条件数变化');
grid on;

% 显示机器人动画
subplot(2,2,[3,4]);
title('机器人运动动画');
for i = 1:num_points
    show(ur5, joint_configs{i}, 'PreservePlot', false);
    hold on;
    plot3(trajectory_points(:,1), trajectory_points(:,2), trajectory_points(:,3), 'b-');
    if ismember(i, singular_points)
        plot3(trajectory_points(i,1), trajectory_points(i,2), trajectory_points(i,3), ...
              'ro', 'MarkerSize', 15);
    end
    drawnow;
    pause(0.01);
end

%% 输出分析结果
fprintf('\n轨迹分析完成:\n');
fprintf('总轨迹点数: %d\n', num_points);
fprintf('检测到的奇异点数: %d\n', length(singular_points));
fprintf('最大条件数: %f\n', max(condition_numbers));
fprintf('最小条件数: %f\n', min(condition_numbers));

% 保存结果
save('trajectory_analysis_results.mat', 'trajectory_points', 'singular_points', ...
     'condition_numbers', 'joint_configs');
