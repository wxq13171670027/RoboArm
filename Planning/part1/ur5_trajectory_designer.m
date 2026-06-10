function ur5_trajectory_designer
    % 创建主窗口
    fig = uifigure('Name', 'UR5轨迹设计器', 'Position', [100 100 1200 800]);
    
    % 创建左侧面板用于控制和输入
    control_panel = uipanel(fig, 'Position', [10 10 280 780], 'Title', '控制面板');
    
    % 创建标签页容器
    tabgp = uitabgroup(fig, 'Position', [300 10 890 780]);
    
    % 创建3D视图标签页
    view_tab = uitab(tabgp, 'Title', '机器人工作空间');
    
    % 创建用于渲染机器人的axes并直接嵌入view_tab
    robot_ax = axes('Parent', view_tab, 'Position', [0.05 0.05 0.9 0.9]);
    hold(robot_ax, 'on');
    grid(robot_ax, 'on');
    axis(robot_ax, 'equal');
    xlabel(robot_ax, 'X (m)');
    ylabel(robot_ax, 'Y (m)');
    zlabel(robot_ax, 'Z (m)');
    view(robot_ax, [135 30]);
    
    % 启用3D旋转和点击功能
    rotate3d(robot_ax, 'on');
    
    % 添加点击事件监听器
    set(robot_ax, 'ButtonDownFcn', @axesClickCallback);
    
    % 设置坐标轴范围
    axis_limit = 1;
    xlim(robot_ax, [-1 1]);
    ylim(robot_ax, [-1 1]);
    zlim(robot_ax, [0 1.2]);
    
    % 加载UR5机器人模型
    ur5 = loadrobot('universalUR5', 'DataFormat', 'row');
    
    % 设置初始配置
    config = homeConfiguration(ur5);
    initial_joints = [0, -pi/2, 0, -pi/2, 0, 0];
    for i = 1:6
        config(i) = initial_joints(i);
    end
    
    % 显示机器人
    figure_handle = figure('Visible', 'off');  % 创建隐藏的figure
    temp_ax = axes(figure_handle);
    show(ur5, config, 'Parent', temp_ax, 'PreservePlot', false);
    
    % 复制图形对象到主界面
    copyobj(get(temp_ax, 'Children'), robot_ax);
    close(figure_handle);  % 关闭临时figure
    
    % 创建UR5实际工作空间
    % UR5 DH参数 (单位: 米)
    d1 = 0.0892;     % 基座到肩部高度
    a2 = -0.425;     % 上臂长度
    a3 = -0.392;     % 前臂长度
    d4 = 0.109;      % 腕部偏置
    d5 = 0.095;      % 腕部到工具端长度
    d6 = 0.082;      % 工具端长度
    
    % 计算实际工作空间参数
    reach_max = abs(a2) + abs(a3) + abs(d4) + abs(d5) + abs(d6);  % 最大臂展约1.1米
    reach_min = max(0.15, abs(d4));  % 最小工作半径
    z_max = d1 + reach_max;          % 最大高度约1.2米
    z_min = -0.2;                    % 最小高度（考虑下方可达空间）
    
    % 生成工作空间点云
    n_points = 50;
    [theta, phi] = meshgrid(linspace(0, 2*pi, n_points), linspace(-pi/2, pi/2, n_points));
    
    % 外部边界 - 使用更大的半径
    R = reach_max;
    X_outer = R * cos(phi) .* cos(theta);
    Y_outer = R * cos(phi) .* sin(theta);
    Z_outer = R * sin(phi) + d1;  % 考虑基座高度偏移
    
    % 绘制外部工作空间边界
    surf(robot_ax, X_outer, Y_outer, Z_outer, ...
        'FaceAlpha', 0.1, 'EdgeAlpha', 0.05, ...
        'FaceColor', [0.8 0.8 1], 'EdgeColor', [0.7 0.7 1]);
    
    % 内部边界
    R = reach_min;
    X_inner = R * cos(phi) .* cos(theta);
    Y_inner = R * cos(phi) .* sin(theta);
    Z_inner = R * sin(phi) + d1;
    
    % 绘制内部工作空间边界
    surf(robot_ax, X_inner, Y_inner, Z_inner, ...
        'FaceAlpha', 0.1, 'EdgeAlpha', 0.05, ...
        'FaceColor', [1 0.8 0.8], 'EdgeColor', [1 0.7 0.7]);
    
    % 添加高度限制平面
    [X_disk, Y_disk] = meshgrid(linspace(-reach_max, reach_max, n_points));
    Z_top = ones(size(X_disk)) * z_max;
    Z_bottom = ones(size(X_disk)) * z_min;
    
    % 绘制顶部和底部平面
    surf(robot_ax, X_disk, Y_disk, Z_top, ...
        'FaceAlpha', 0.1, 'EdgeAlpha', 0.05, ...
        'FaceColor', [0.8 1 0.8], 'EdgeColor', [0.7 1 0.7]);
    surf(robot_ax, X_disk, Y_disk, Z_bottom, ...
        'FaceAlpha', 0.1, 'EdgeAlpha', 0.05, ...
        'FaceColor', [0.8 1 0.8], 'EdgeColor', [0.7 1 0.7]);
    
    % 更新坐标轴范围以适应新的工作空间
    axis_limit = reach_max + 0.1;  % 添加一些边距
    xlim(robot_ax, [-axis_limit axis_limit]);
    ylim(robot_ax, [-axis_limit axis_limit]);
    zlim(robot_ax, [z_min z_max + 0.1]);
    
    % 添加坐标轴标签和图例
    title(robot_ax, 'UR5机器人工作空间');
    legend(robot_ax, '外部边界', '内部边界', '高度限制');
    
    % 启用3D旋转
    rotate3d(robot_ax, 'on');
    
    % 创建轨迹类型选择下拉菜单
    uilabel(control_panel, 'Position', [10 730 100 22], 'Text', '轨迹类型:');
    trajectory_type = uidropdown(control_panel, ...
        'Position', [10 700 260 30], ...
        'Items', {'圆形轨迹', '直线轨迹', '矩形轨迹', '螺旋线轨迹', 'S形轨迹', ...
                 '波浪形轨迹', '八字形轨迹'}, ...
        'Value', '圆形轨迹', ...
        'ValueChangedFcn', @updateTrajectoryType);
    
    % 圆形轨迹参数面板
    circle_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', '圆形轨迹参数');
    
    % 圆形轨迹参数控件
    uilabel(circle_panel, 'Position', [10 130 100 22], 'Text', '圆心 X (m):');
    center_x = uispinner(circle_panel, 'Position', [120 130 130 22], ...
        'Value', 0.4, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(circle_panel, 'Position', [10 100 100 22], 'Text', '圆心 Y (m):');
    center_y = uispinner(circle_panel, 'Position', [120 100 130 22], ...
        'Value', 0, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(circle_panel, 'Position', [10 70 100 22], 'Text', '圆心 Z (m):');
    center_z = uispinner(circle_panel, 'Position', [120 70 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    uilabel(circle_panel, 'Position', [10 40 100 22], 'Text', '半径 (m):');
    radius = uispinner(circle_panel, 'Position', [120 40 130 22], ...
        'Value', 0.15, 'Limits', [0.05 0.3], 'Step', 0.05);
    
    % 直线轨迹参数面板
    line_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', '直线轨迹参数', 'Visible', 'off');
    
    % 起点
    uilabel(line_panel, 'Position', [10 130 100 22], 'Text', '起点 X (m):');
    start_x = uispinner(line_panel, 'Position', [120 130 130 22], ...
        'Value', 0.3, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(line_panel, 'Position', [10 100 100 22], 'Text', '起点 Y (m):');
    start_y = uispinner(line_panel, 'Position', [120 100 130 22], ...
        'Value', 0, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(line_panel, 'Position', [10 70 100 22], 'Text', '起点 Z (m):');
    start_z = uispinner(line_panel, 'Position', [120 70 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    % 终点
    uilabel(line_panel, 'Position', [10 40 100 22], 'Text', '终点 X (m):');
    end_x = uispinner(line_panel, 'Position', [120 40 130 22], ...
        'Value', 0.5, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(line_panel, 'Position', [10 10 100 22], 'Text', '终点 Y (m):');
    end_y = uispinner(line_panel, 'Position', [120 10 130 22], ...
        'Value', 0.3, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(line_panel, 'Position', [10 -20 100 22], 'Text', '终点 Z (m):');
    end_z = uispinner(line_panel, 'Position', [120 -20 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    % 矩形轨迹参数面板
    rect_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', '矩形轨迹参数', 'Visible', 'off');
    
    uilabel(rect_panel, 'Position', [10 130 100 22], 'Text', '中心 X (m):');
    rect_x = uispinner(rect_panel, 'Position', [120 130 130 22], ...
        'Value', 0.4, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(rect_panel, 'Position', [10 100 100 22], 'Text', '中心 Y (m):');
    rect_y = uispinner(rect_panel, 'Position', [120 100 130 22], ...
        'Value', 0, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(rect_panel, 'Position', [10 70 100 22], 'Text', '中心 Z (m):');
    rect_z = uispinner(rect_panel, 'Position', [120 70 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    uilabel(rect_panel, 'Position', [10 40 100 22], 'Text', '长度 (m):');
    rect_length = uispinner(rect_panel, 'Position', [120 40 130 22], ...
        'Value', 0.2, 'Limits', [0.05 0.4], 'Step', 0.05);
    
    uilabel(rect_panel, 'Position', [10 10 100 22], 'Text', '宽度 (m):');
    rect_width = uispinner(rect_panel, 'Position', [120 10 130 22], ...
        'Value', 0.15, 'Limits', [0.05 0.3], 'Step', 0.05);
    
    % 添加螺旋线轨迹参数面板
    helix_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', '螺旋线轨迹参数', 'Visible', 'off');
    
    uilabel(helix_panel, 'Position', [10 130 100 22], 'Text', '中心 X (m):');
    helix_x = uispinner(helix_panel, 'Position', [120 130 130 22], ...
        'Value', 0.4, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(helix_panel, 'Position', [10 100 100 22], 'Text', '中心 Y (m):');
    helix_y = uispinner(helix_panel, 'Position', [120 100 130 22], ...
        'Value', 0, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(helix_panel, 'Position', [10 70 100 22], 'Text', '起始 Z (m):');
    helix_z = uispinner(helix_panel, 'Position', [120 70 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    uilabel(helix_panel, 'Position', [10 40 100 22], 'Text', '半径 (m):');
    helix_radius = uispinner(helix_panel, 'Position', [120 40 130 22], ...
        'Value', 0.15, 'Limits', [0.05 0.3], 'Step', 0.05);
    
    uilabel(helix_panel, 'Position', [10 10 100 22], 'Text', '高度 (m):');
    helix_height = uispinner(helix_panel, 'Position', [120 10 130 22], ...
        'Value', 0.2, 'Limits', [0.05 0.4], 'Step', 0.05);

    % 添加S形轨迹参数面板
    s_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', 'S形轨迹参数', 'Visible', 'off');
    
    uilabel(s_panel, 'Position', [10 130 100 22], 'Text', '起点 X (m):');
    s_start_x = uispinner(s_panel, 'Position', [120 130 130 22], ...
        'Value', 0.3, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(s_panel, 'Position', [10 100 100 22], 'Text', '起点 Y (m):');
    s_start_y = uispinner(s_panel, 'Position', [120 100 130 22], ...
        'Value', -0.2, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(s_panel, 'Position', [10 70 100 22], 'Text', '终点 X (m):');
    s_end_x = uispinner(s_panel, 'Position', [120 70 130 22], ...
        'Value', 0.5, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(s_panel, 'Position', [10 40 100 22], 'Text', '终点 Y (m):');
    s_end_y = uispinner(s_panel, 'Position', [120 40 130 22], ...
        'Value', 0.2, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(s_panel, 'Position', [10 10 100 22], 'Text', '高度 Z (m):');
    s_height = uispinner(s_panel, 'Position', [120 10 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);

    % 添加波浪形轨迹参数面板
    wave_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', '波浪形轨迹参数', 'Visible', 'off');
    
    uilabel(wave_panel, 'Position', [10 130 100 22], 'Text', '中心 X (m):');
    wave_x = uispinner(wave_panel, 'Position', [120 130 130 22], ...
        'Value', 0.4, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(wave_panel, 'Position', [10 100 100 22], 'Text', '中心 Y (m):');
    wave_y = uispinner(wave_panel, 'Position', [120 100 130 22], ...
        'Value', 0, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(wave_panel, 'Position', [10 70 100 22], 'Text', '高度 Z (m):');
    wave_z = uispinner(wave_panel, 'Position', [120 70 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    uilabel(wave_panel, 'Position', [10 40 100 22], 'Text', '波长 (m):');
    wave_length = uispinner(wave_panel, 'Position', [120 40 130 22], ...
        'Value', 0.3, 'Limits', [0.1 0.5], 'Step', 0.05);
    
    uilabel(wave_panel, 'Position', [10 10 100 22], 'Text', '振幅 (m):');
    wave_amp = uispinner(wave_panel, 'Position', [120 10 130 22], ...
        'Value', 0.1, 'Limits', [0.05 0.2], 'Step', 0.02);

    % 添加八字形轨迹参数面板
    eight_panel = uipanel(control_panel, 'Position', [10 500 260 190], 'Title', '八字形轨迹参数', 'Visible', 'off');
    
    uilabel(eight_panel, 'Position', [10 130 100 22], 'Text', '中心 X (m):');
    eight_x = uispinner(eight_panel, 'Position', [120 130 130 22], ...
        'Value', 0.4, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(eight_panel, 'Position', [10 100 100 22], 'Text', '中心 Y (m):');
    eight_y = uispinner(eight_panel, 'Position', [120 100 130 22], ...
        'Value', 0, 'Limits', [-0.85 0.85], 'Step', 0.05);
    
    uilabel(eight_panel, 'Position', [10 70 100 22], 'Text', '高度 Z (m):');
    eight_z = uispinner(eight_panel, 'Position', [120 70 130 22], ...
        'Value', 0.2, 'Limits', [0 0.85], 'Step', 0.05);
    
    uilabel(eight_panel, 'Position', [10 40 100 22], 'Text', '长度 (m):');
    eight_length = uispinner(eight_panel, 'Position', [120 40 130 22], ...
        'Value', 0.3, 'Limits', [0.1 0.5], 'Step', 0.05);
    
    uilabel(eight_panel, 'Position', [10 10 100 22], 'Text', '宽度 (m):');
    eight_width = uispinner(eight_panel, 'Position', [120 10 130 22], ...
        'Value', 0.15, 'Limits', [0.05 0.3], 'Step', 0.05);
    
    % 采样点数（所有轨迹共用）
    uilabel(control_panel, 'Position', [10 460 100 22], 'Text', '采样点数:');
    num_points = uispinner(control_panel, 'Position', [120 460 130 22], ...
        'Value', 100, 'Limits', [10 200], 'Step', 10);
    
    % 预览按钮
    preview_btn = uibutton(control_panel, 'Position', [10 420 260 30], ...
        'Text', '预览轨迹', ...
        'ButtonPushedFcn', @previewTrajectory);
    
    % 验证按钮
    validate_btn = uibutton(control_panel, 'Position', [10 380 260 30], ...
        'Text', '验证轨迹可行性', ...
        'ButtonPushedFcn', @validateTrajectory);
    
    % 执行按钮
    execute_btn = uibutton(control_panel, 'Position', [10 340 260 30], ...
        'Text', '分析轨迹', ...
        'ButtonPushedFcn', @executeTrajectory);
    
    % 添加导出按钮
    export_btn = uibutton(control_panel, 'Position', [10 300 260 30], ...
        'Text', '导出轨迹', ...
        'ButtonPushedFcn', @exportTrajectory);
    
    % 打开处理工具按钮
    process_btn = uibutton(control_panel, 'Position', [10 260 260 30], ...
        'Text', '打开轨迹处理工具', ...
        'ButtonPushedFcn', @(~,~)trajectory_processor_v2());
    
    % 存储当前轨迹点
    trajectory_points = [];
    joint_configs = {};
    plot_handle = [];
    
    % 预览轨迹的回调函数
    function previewTrajectory(~, ~)
        % 保持当前视角
        current_view = get(robot_ax, 'View');
        
        % 清除之前的轨迹
        if ~isempty(plot_handle)
            for i = 1:numel(plot_handle)
                if isvalid(plot_handle(i))
                    delete(plot_handle(i));
                end
            end
        end
        plot_handle = [];  % 重置句柄数组
        
        % 重新设置图形属性
        hold(robot_ax, 'on');
        grid(robot_ax, 'on');
        axis(robot_ax, 'equal');
        
        % 设置固定的坐标轴范围
        axis_limit = 1;
        xlim(robot_ax, [-1 1]);
        ylim(robot_ax, [-1 1]);
        zlim(robot_ax, [0 1.2]);
        
        % 生成轨迹点
        switch trajectory_type.Value
            case '圆形轨迹'
                % 生成闭合的圆形轨迹
                theta = linspace(0, 2*pi, num_points.Value+1);  % +1是为了闭合
                theta = theta(1:end-1);  % 去掉重复的最后一个点
                center = [center_x.Value, center_y.Value, center_z.Value];
                r = radius.Value;
                trajectory_points = zeros(num_points.Value, 3);
                for i = 1:num_points.Value
                    trajectory_points(i,:) = center + r * [cos(theta(i)), sin(theta(i)), 0];
                end
                
            case '直线轨迹'
                start_point = [start_x.Value, start_y.Value, start_z.Value];
                end_point = [end_x.Value, end_y.Value, end_z.Value];
                t = linspace(0, 1, num_points.Value);
                trajectory_points = zeros(num_points.Value, 3);
                for i = 1:num_points.Value
                    trajectory_points(i,:) = start_point + t(i)*(end_point - start_point);
                end
                
            case '矩形轨迹'
                center = [rect_x.Value, rect_y.Value, rect_z.Value];
                length = rect_length.Value;
                width = rect_width.Value;
                
                % 计算每条边的点数
                total_perimeter = 2 * (length + width);
                points_per_unit = num_points.Value / total_perimeter;
                nl = round(length * points_per_unit);  % 长边点数
                nw = round(width * points_per_unit);   % 宽边点数
                
                % 生成四条边的点
                t_length = linspace(0, 1, nl);
                t_width = linspace(0, 1, nw);
                
                % 定义四个角点
                corners = [
                    center + [-length/2, -width/2, 0];  % 左下
                    center + [length/2, -width/2, 0];   % 右下
                    center + [length/2, width/2, 0];    % 右上
                    center + [-length/2, width/2, 0];   % 左上
                ];
                
                % 生成轨迹点
                trajectory_points = [];
                
                % 下边
                for t = t_length
                    point = corners(1,:) + t * (corners(2,:) - corners(1,:));
                    trajectory_points = [trajectory_points; point];
                end
                
                % 右边
                for t = t_width
                    point = corners(2,:) + t * (corners(3,:) - corners(2,:));
                    trajectory_points = [trajectory_points; point];
                end
                
                % 上边
                for t = t_length
                    point = corners(3,:) + t * (corners(4,:) - corners(3,:));
                    trajectory_points = [trajectory_points; point];
                end
                
                % 左边
                for t = t_width
                    point = corners(4,:) + t * (corners(1,:) - corners(4,:));
                    trajectory_points = [trajectory_points; point];
                end
                
            case '螺旋线轨迹'
                % 生成螺旋线轨迹
                center = [helix_x.Value, helix_y.Value, helix_z.Value];
                radius = helix_radius.Value;
                height = helix_height.Value;
                
                % 计算螺旋线参数
                t = linspace(0, 4*pi, num_points.Value);  % 两圈螺旋线
                x = center(1) + radius * cos(t);
                y = center(2) + radius * sin(t);
                z = center(3) + height * t/(4*pi);  % 线性增加高度
                
                trajectory_points = [x', y', z'];
                
            case 'S形轨迹'
                % 生成S形轨迹
                start = [s_start_x.Value, s_start_y.Value, s_height.Value];
                ending = [s_end_x.Value, s_end_y.Value, s_height.Value];
                
                % 使用贝塞尔曲线生成S形
                t = linspace(0, 1, num_points.Value);
                control1 = start + [0.33*(ending(1)-start(1)), -0.2, 0];
                control2 = ending + [-0.33*(ending(1)-start(1)), 0.2, 0];
                
                trajectory_points = zeros(num_points.Value, 3);
                for i = 1:num_points.Value
                    tt = t(i);
                    trajectory_points(i,:) = (1-tt)^3 * start + ...
                        3*tt*(1-tt)^2 * control1 + ...
                        3*tt^2*(1-tt) * control2 + ...
                        tt^3 * ending;
                end

            case '波浪形轨迹'
                % 生成波浪形轨迹
                center = [wave_x.Value, wave_y.Value, wave_z.Value];
                wavelength = wave_length.Value;
                amplitude = wave_amp.Value;
                
                t = linspace(-pi, pi, num_points.Value);
                x = center(1) + wavelength * t/(2*pi);
                y = center(2) + amplitude * sin(t);
                z = center(3) * ones(size(t));
                
                trajectory_points = [x', y', z'];

            case '八字形轨迹'
                % 生成八字形轨迹
                center = [eight_x.Value, eight_y.Value, eight_z.Value];
                length = eight_length.Value;
                width = eight_width.Value;
                
                t = linspace(0, 2*pi, num_points.Value);
                x = center(1) + length * sin(t);
                y = center(2) + width * sin(t) .* cos(t);
                z = center(3) * ones(size(t));
                
                trajectory_points = [x', y', z'];
        end
        
        % 绘制轨迹
        plot_handle = plot3(robot_ax, trajectory_points(:,1), trajectory_points(:,2), ...
            trajectory_points(:,3), 'b-', 'LineWidth', 2);
        
        % 添加原始采样点的显示
        hold(robot_ax, 'on');
        plot_handle(end+1) = plot3(robot_ax, trajectory_points(:,1), trajectory_points(:,2), ...
            trajectory_points(:,3), 'r.', 'MarkerSize', 10);
        hold(robot_ax, 'off');
        
        % 恢复之前的视角
        view(robot_ax, current_view);
        
        % 更新图形
        drawnow;
    end
    
    % 验证轨迹可行性的回调函数
    function validateTrajectory(~, ~)
        if isempty(trajectory_points)
            uialert(fig, '请先预览轨迹！', '警告', 'Icon', 'warning');
            return;
        end
        
        % 创建进度条
        d = uiprogressdlg(fig, 'Title', '验证轨迹', ...
            'Message', '正在验证轨迹可行性...');
        
        % 创建IK求解器
        ik = inverseKinematics('RigidBodyTree', ur5);
        weights = [1 1 1 1 1 1];
        
        % 验证每个点
        valid_points = 0;
        invalid_points = [];
        
        for i = 1:size(trajectory_points, 1)
            d.Value = i/size(trajectory_points, 1);
            d.Message = sprintf('验证点 %d/%d...', i, size(trajectory_points, 1));
            
            % 创建目标位姿
            tform = trvec2tform(trajectory_points(i,:)) * eul2tform([0, 0, pi/2]);
            
            try
                % 尝试求解IK
                [config_sol, ~] = ik('tool0', tform, weights, config);
                valid_points = valid_points + 1;
            catch
                invalid_points = [invalid_points; i];
            end
        end
        
        close(d);
        
        % 显示结果
        if isempty(invalid_points)
            uialert(fig, sprintf('轨迹完全可行！\n有效点数: %d/%d', ...
                valid_points, size(trajectory_points, 1)), ...
                '验证结果', 'Icon', 'success');
        else
            uialert(fig, sprintf('轨迹部分不可行！\n有效点数: %d/%d\n不可行点索引: %s', ...
                valid_points, size(trajectory_points, 1), ...
                mat2str(invalid_points)), ...
                '验证结果', 'Icon', 'warning');
        end
    end
    
    % 执行轨迹分析的回调函数
    function executeTrajectory(~, ~)
        if isempty(trajectory_points)
            uialert(fig, '请先预览轨迹！', '警告', 'Icon', 'warning');
            return;
        end
        
        % 保存轨迹点和工作空间
        assignin('base', 'custom_trajectory', trajectory_points);
        
        % 运行分析脚本
        analyze_trajectory_singularity_custom(trajectory_points);
    end
    
    % 导出轨迹的回调函数
    function exportTrajectory(~, ~)
        if isempty(trajectory_points)
            uialert(fig, '请先生成轨迹！', '警告', 'Icon', 'warning');
            return;
        end
        
        try
            % 准备导出数据
            export_data = struct();
            export_data.trajectory_points = trajectory_points;
            export_data.joint_configs = joint_configs;
            export_data.trajectory_type = trajectory_type.Value;
            export_data.parameters = struct();
            
            % 保存参数
            switch trajectory_type.Value
                case '圆形轨迹'
                    export_data.parameters.center = [center_x.Value, center_y.Value, center_z.Value];
                    export_data.parameters.radius = radius.Value;
                case '直线轨迹'
                    export_data.parameters.start = [start_x.Value, start_y.Value, start_z.Value];
                    export_data.parameters.end = [end_x.Value, end_y.Value, end_z.Value];
                case '矩形轨迹'
                    export_data.parameters.center = [rect_x.Value, rect_y.Value, rect_z.Value];
                    export_data.parameters.length = rect_length.Value;
                    export_data.parameters.width = rect_width.Value;
                case '螺旋线轨迹'
                    export_data.parameters.center = [helix_x.Value, helix_y.Value, helix_z.Value];
                    export_data.parameters.radius = helix_radius.Value;
                    export_data.parameters.height = helix_height.Value;
                case 'S形轨迹'
                    export_data.parameters.start = [s_start_x.Value, s_start_y.Value, s_height.Value];
                    export_data.parameters.end = [s_end_x.Value, s_end_y.Value, s_height.Value];
                case '波浪形轨迹'
                    export_data.parameters.center = [wave_x.Value, wave_y.Value, wave_z.Value];
                    export_data.parameters.wavelength = wave_length.Value;
                    export_data.parameters.amplitude = wave_amp.Value;
                case '八字形轨迹'
                    export_data.parameters.center = [eight_x.Value, eight_y.Value, eight_z.Value];
                    export_data.parameters.length = eight_length.Value;
                    export_data.parameters.width = eight_width.Value;
            end
            
            % 选择保存位置
            [filename, pathname] = uiputfile({'*.mat', 'MAT文件 (*.mat)'}, '保存轨迹');
            if filename ~= 0
                save(fullfile(pathname, filename), '-struct', 'export_data');
                uialert(fig, '轨迹已成功导出！', '成功', 'Icon', 'success');
            end
            
        catch e
            uialert(fig, ['导出错误: ' e.message], '错误', 'Icon', 'error');
        end
    end
    
    % 打开处理工具按钮的回调函数
    function openProcessor(~, ~)
        trajectory_processor_v2();
    end
    
    % 修改更新轨迹类型的回调函数
    function updateTrajectoryType(~, ~)
        % 隐藏所有参数面板
        circle_panel.Visible = 'off';
        line_panel.Visible = 'off';
        rect_panel.Visible = 'off';
        helix_panel.Visible = 'off';
        s_panel.Visible = 'off';
        wave_panel.Visible = 'off';
        eight_panel.Visible = 'off';
        
        % 显示选中类型的参数面板
        switch trajectory_type.Value
            case '圆形轨迹'
                circle_panel.Visible = 'on';
            case '直线轨迹'
                line_panel.Visible = 'on';
            case '矩形轨迹'
                rect_panel.Visible = 'on';
            case '螺旋线轨迹'
                helix_panel.Visible = 'on';
            case 'S形轨迹'
                s_panel.Visible = 'on';
            case '波浪形轨迹'
                wave_panel.Visible = 'on';
            case '八字形轨迹'
                eight_panel.Visible = 'on';
        end
    end
end
