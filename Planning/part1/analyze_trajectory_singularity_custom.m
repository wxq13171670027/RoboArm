function analyze_trajectory_singularity_custom(trajectory_points)
    % 创建UR5机器人模型
    ur5 = loadrobot('universalUR5', 'DataFormat', 'row');
    
    % 定义UR5的关节限制
    joint_limits = [-2*pi, 2*pi;    % Base
                   -2*pi, 2*pi;    % Shoulder
                   -pi, pi;        % Elbow
                   -2*pi, 2*pi;    % Wrist 1
                   -2*pi, 2*pi;    % Wrist 2
                   -2*pi, 2*pi];   % Wrist 3
    
    % 计算轨迹上每个点的逆解和奇异性
    num_points = size(trajectory_points, 1);
    singular_threshold = 1e-3;
    condition_numbers = zeros(num_points, 1);
    min_singular_values = zeros(num_points, 1);
    manipulability_measures = zeros(num_points, 1);
    singular_points = [];
    joint_configs = cell(num_points, 1);
    
    % 创建IK求解器
    ik = inverseKinematics('RigidBodyTree', ur5);
    weights = [1 1 1 1 1 1];  % 权重向量
    
    % 设置初始构型
    initial_joints = [0, -pi/2, 0, -pi/2, 0, 0];  % UR5的标准初始位置
    initial_config = homeConfiguration(ur5);
    for i = 1:numel(initial_config)
        initial_config(i) = initial_joints(i);
    end
    
    % 创建图形窗口
    fig = figure('Name', '轨迹奇异性分析', 'NumberTitle', 'off', 'Position', [100 100 1200 600]);
    
    % 创建子图布局
    subplot(1, 3, 1);
    plot3(trajectory_points(:,1), trajectory_points(:,2), trajectory_points(:,3), ...
        'b-', 'LineWidth', 2, 'DisplayName', '规划轨迹');
    hold on;
    scatter3(trajectory_points(:,1), trajectory_points(:,2), trajectory_points(:,3), ...
        50, 'r', 'filled', 'DisplayName', '轨迹点');
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('轨迹路径');
    legend('show');
    view(3);
    axis equal;
    
    fprintf('开始分析轨迹...\n');
    fprintf('总计 %d 个轨迹点\n', num_points);
    
    % 分析每个轨迹点
    for i = 1:num_points
        try
            % 创建目标位姿（保持末端执行器朝下）
            current_point = trajectory_points(i,:);
            
            % 计算末端执行器姿态
            if i > 1
                % 计算移动方向作为参考
                direction = trajectory_points(i,:) - trajectory_points(max(1,i-1),:);
                direction = direction / norm(direction);
                
                % 创建姿态矩阵（工具始终朝下，但可以根据移动方向调整）
                z_axis = [0; 0; -1];  % 工具朝下
                y_axis = cross(z_axis, direction');  % 垂直于移动方向
                if norm(y_axis) < 1e-6
                    y_axis = [0; 1; 0];  % 默认值
                else
                    y_axis = y_axis / norm(y_axis);
                end
                x_axis = cross(y_axis, z_axis);
                R = [x_axis, y_axis, z_axis];
            else
                % 第一个点使用默认姿态
                R = [1 0 0; 0 1 0; 0 0 -1];  % 工具朝下
            end
            
            % 创建目标变换矩阵
            tform = [R, current_point'; 0 0 0 1];
            
            % 计算逆解
            if i == 1
                [config, info] = ik('tool0', tform, weights, initial_config);
            else
                prev_config = joint_configs{i-1};
                if isempty(prev_config)
                    prev_config = initial_config;
                end
                [config, info] = ik('tool0', tform, weights, prev_config);
            end
            
            % 存储配置
            joint_configs{i} = config;
            
            % 验证解的正确性
            fk = getTransform(ur5, config, 'tool0');
            pos_error = norm(tform2trvec(fk) - trajectory_points(i,:));
            
            % 计算雅可比矩阵和奇异性
            J = geometricJacobian(ur5, config, 'tool0');
            s = svd(J);
            min_singular_values(i) = min(s);
            condition_numbers(i) = max(s)/min(s);
            manipulability_measures(i) = sqrt(det(J*J'));
            
            if min(s) < singular_threshold
                singular_points = [singular_points; i];
                fprintf('在轨迹点 %d 处检测到奇异点\n', i);
                fprintf('条件数: %f\n', condition_numbers(i));
            end
            
        catch ME
            fprintf('在轨迹点 %d 处无法求解逆运动学:\n', i);
            fprintf('错误信息: %s\n', ME.message);
            if i > 1
                joint_configs{i} = joint_configs{i-1};
            else
                joint_configs{i} = initial_config;
            end
            min_singular_values(i) = NaN;
            condition_numbers(i) = NaN;
            manipulability_measures(i) = NaN;
        end
    end
    
    % 绘制奇异值分析
    subplot(1, 3, 2);
    plot(1:num_points, min_singular_values, 'b-', 'LineWidth', 2, 'DisplayName', '最小奇异值');
    hold on;
    yline(singular_threshold, 'r--', '奇异值阈值', 'DisplayName', '奇异阈值');
    if ~isempty(singular_points)
        scatter(singular_points, min_singular_values(singular_points), ...
            100, 'r', 'filled', 'DisplayName', '奇异点');
    end
    grid on;
    xlabel('轨迹点索引');
    ylabel('最小奇异值');
    title('奇异值分析');
    legend('show');
    
    % 绘制条件数分析
    subplot(1, 3, 3);
    plot(1:num_points, condition_numbers, 'r-', 'LineWidth', 2, 'DisplayName', '条件数');
    hold on;
    if ~isempty(singular_points)
        scatter(singular_points, condition_numbers(singular_points), ...
            100, 'r', 'filled', 'DisplayName', '奇异点');
    end
    grid on;
    xlabel('轨迹点索引');
    ylabel('条件数');
    title('条件数分析');
    legend('show');
    
    % 输出分析结果
    fprintf('\n轨迹分析完成:\n');
    fprintf('总轨迹点数: %d\n', num_points);
    fprintf('检测到的奇异点数: %d\n', length(singular_points));
    fprintf('最大条件数: %f\n', max(condition_numbers));
    fprintf('最小条件数: %f\n', min(condition_numbers));
    
    % 保存结果
    save('trajectory_analysis_results.mat', 'trajectory_points', 'singular_points', ...
        'condition_numbers', 'joint_configs', 'min_singular_values', ...
        'manipulability_measures');
end
