function trajectory_processor_v2
    % 创建主窗口
    fig = uifigure('Name', '轨迹处理工具 V2', ...
        'Position', [100 100 1400 800], ...
        'Color', [0.94 0.94 0.94]);
    
    % 创建主网格布局
    main_grid = uigridlayout(fig, [1 3]);
    main_grid.ColumnWidth = {'1x', '1.4x', '1.6x'};
    main_grid.Padding = [10 10 10 10];
    main_grid.RowSpacing = 10;
    main_grid.ColumnSpacing = 15;
    
    % === 左侧面板：轨迹导入/导出 ===
    left_panel = uipanel(main_grid, 'Title', '轨迹导入/导出', ...
        'FontWeight', 'bold', 'FontSize', 12);
    left_layout = uigridlayout(left_panel, [7 1]);
    left_layout.RowHeight = {'fit', 'fit', '1x', 'fit', 'fit', 'fit', 'fit'};
    left_layout.Padding = [10 10 10 10];
    
    % 导入部分
    import_section = uipanel(left_layout, 'Title', '导入设置');
    import_grid = uigridlayout(import_section, [2 1]);
    import_grid.RowHeight = {'fit', 'fit'};
    
    uibutton(import_grid, 'Text', '导入轨迹文件', ...
        'FontWeight', 'bold', ...
        'ButtonPushedFcn', @importTrajectory);
    
    file_info = uilabel(import_grid, 'Text', '当前文件: 无', ...
        'WordWrap', 'on');
    
    % 导出部分
    export_section = uipanel(left_layout, 'Title', '导出设置');
    export_grid = uigridlayout(export_section, [4 1]);
    export_grid.RowHeight = {'fit', 'fit', 'fit', 'fit'};
    
    % 文件格式选择
    format_panel = uipanel(export_grid, 'Title', '文件格式');
    format_layout = uigridlayout(format_panel, [3 1]);
    format_layout.RowHeight = {'fit', 'fit', 'fit'};
    
    format_group = uibuttongroup(format_layout, ...
        'SelectionChangedFcn', @(~,~)disp('格式已更改'));
    
    uiradiobutton(format_group, 'Text', 'MAT文件 (.mat)', ...
        'Position', [10 70 150 20], 'Value', true);
    uiradiobutton(format_group, 'Text', 'CSV文件 (.csv)', ...
        'Position', [10 40 150 20]);
    uiradiobutton(format_group, 'Text', '文本文件 (.txt)', ...
        'Position', [10 10 150 20]);
    
    % 导出选项
    options_panel = uipanel(export_grid, 'Title', '导出选项');
    options_layout = uigridlayout(options_panel, [3 1]);
    options_layout.RowHeight = {'fit', 'fit', 'fit'};
    
    uicheckbox(options_layout, 'Text', '时间戳', 'Value', true);
    uicheckbox(options_layout, 'Text', '关节角度', 'Value', true);
    uicheckbox(options_layout, 'Text', '末端位姿', 'Value', true);
    
    % 导出按钮
    uibutton(export_grid, 'Text', '导出轨迹', ...
        'FontWeight', 'bold', ...
        'ButtonPushedFcn', @exportTrajectory);
    
    % === 自定义点管理 ===
    custom_points_panel = uipanel(left_panel, 'Title', '自定义点管理', ...
        'Position', [0.05 0.6 0.9 0.3]);
    
    % 添加点按钮
    add_point_btn = uibutton(custom_points_panel, 'Text', '添加当前点', ...
        'Position', [10 120 100 22], ...
        'ButtonPushedFcn', @addCurrentPoint);
    
    % 删除点按钮
    delete_point_btn = uibutton(custom_points_panel, 'Text', '删除选中点', ...
        'Position', [120 120 100 22], ...
        'ButtonPushedFcn', @deleteSelectedPoint);
    
    % 点列表
    points_list = uilistbox(custom_points_panel, ...
        'Position', [10 10 210 100], ...
        'ValueChangedFcn', @pointSelected);
    
    % === 中间面板：参数设置 ===
    middle_panel = uipanel(main_grid, 'Title', '轨迹处理参数', ...
        'FontWeight', 'bold', 'FontSize', 12);
    middle_layout = uigridlayout(middle_panel, [10 2]);
    middle_layout.ColumnWidth = {'fit', '1x'};
    middle_layout.RowHeight = {'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', '1x'};
    middle_layout.Padding = [10 10 10 10];
    
    % 参数标签和输入框
    params = struct();
    
    % 采样方式
    uilabel(middle_layout, 'Text', '采样方式:', 'FontWeight', 'bold');
    params.sampling = uidropdown(middle_layout, ...
        'Items', {'等时间间隔', '等弧长间隔'}, ...
        'Value', '等时间间隔');
    
    % 采样间隔
    uilabel(middle_layout, 'Text', '采样间隔:', 'FontWeight', 'bold');
    params.interval = uispinner(middle_layout, 'Value', 0.01, ...
        'Limits', [0.001 1], 'Step', 0.001);
    
    % 插值方法
    uilabel(middle_layout, 'Text', '插值方法:', 'FontWeight', 'bold');
    params.interp = uidropdown(middle_layout, ...
        'Items', {'线性插值', '三次样条', '五次样条'}, ...
        'Value', '线性插值');
    
    % 平滑参数
    uilabel(middle_layout, 'Text', '平滑参数:', 'FontWeight', 'bold');
    params.smooth = uispinner(middle_layout, 'Value', 0.5, ...
        'Limits', [0 1], 'Step', 0.1);
    
    % 最大速度
    uilabel(middle_layout, 'Text', '最大速度 (m/s):', 'FontWeight', 'bold');
    params.max_vel = uispinner(middle_layout, 'Value', 1, ...
        'Limits', [0.1 10], 'Step', 0.1);
    
    % 最大加速度
    uilabel(middle_layout, 'Text', '最大加速度 (m/s²):', 'FontWeight', 'bold');
    params.max_acc = uispinner(middle_layout, 'Value', 2, ...
        'Limits', [0.1 20], 'Step', 0.1);
    
    % 处理按钮
    uilabel(middle_layout, 'Text', '', 'FontWeight', 'bold');  % 空标签用于对齐
    process_btn = uibutton(middle_layout, 'Text', '处理轨迹', ...
        'FontWeight', 'bold', ...
        'ButtonPushedFcn', @processTrajectory);
    
    % === 右侧面板：可视化 ===
    right_panel = uipanel(main_grid, 'Title', '轨迹可视化', ...
        'FontWeight', 'bold', 'FontSize', 12);
    right_layout = uigridlayout(right_panel, [6 1]);
    right_layout.RowHeight = {'4x', 'fit'};
    right_layout.Padding = [10 10 10 10];
    
    % 创建轨迹显示区域
    ax_trajectory = uiaxes(right_layout);
    ax_trajectory.Layout.Row = 1;
    grid(ax_trajectory, 'on');
    box(ax_trajectory, 'on');
    view(ax_trajectory, [45 30]);
    xlabel(ax_trajectory, 'X (m)');
    ylabel(ax_trajectory, 'Y (m)');
    zlabel(ax_trajectory, 'Z (m)');
    title(ax_trajectory, '轨迹可视化');
    
    % 图例面板
    legend_panel = uipanel(right_layout, 'Title', '显示选项');
    legend_layout = uigridlayout(legend_panel, [2 3]);
    legend_layout.ColumnWidth = {'1x', '1x', '1x'};
    
    % 显示选项复选框
    display_options = struct();
    display_options.original_trajectory = uicheckbox(legend_layout, 'Text', '原始轨迹', 'Value', true);
    display_options.processed_trajectory = uicheckbox(legend_layout, 'Text', '处理后轨迹', 'Value', true);
    display_options.sample_points = uicheckbox(legend_layout, 'Text', '采样点', 'Value', true);
    display_options.velocity_vectors = uicheckbox(legend_layout, 'Text', '速度向量', 'Value', false);
    display_options.acceleration_vectors = uicheckbox(legend_layout, 'Text', '加速度向量', 'Value', false);
    display_options.key_points = uicheckbox(legend_layout, 'Text', '关键点', 'Value', true);
    
    % 存储数据
    data = struct();
    data.original_trajectory = [];
    data.processed_trajectory = [];
    data.custom_points = [];
    data.points_list = points_list;
    
    % === 回调函数 ===
    function importTrajectory(~, ~)
        [filename, pathname] = uigetfile({'*.mat', 'MAT文件 (*.mat)'; ...
            '*.csv', 'CSV文件 (*.csv)'; ...
            '*.txt', '文本文件 (*.txt)'});
        if isequal(filename, 0)
            return;
        end
        
        try
            % 读取文件
            fullpath = fullfile(pathname, filename);
            [~, ~, ext] = fileparts(filename);
            
            switch lower(ext)
                case '.mat'
                    loaded = load(fullpath);
                    % 检查所有可能的变量名
                    if isfield(loaded, 'trajectory_points')
                        data.original_trajectory = loaded.trajectory_points;
                    elseif isfield(loaded, 'trajectory')
                        data.original_trajectory = loaded.trajectory;
                    elseif isfield(loaded, 'ax')
                        % 从ur5_trajectory_designer导出的数据
                        if isfield(loaded.ax, 'trajectory_points')
                            data.original_trajectory = loaded.ax.trajectory_points;
                        else
                            error('无法找到轨迹数据');
                        end
                    else
                        % 获取MAT文件中的第一个数值数组
                        fields = fieldnames(loaded);
                        found = false;
                        for i = 1:length(fields)
                            if isnumeric(loaded.(fields{i})) && size(loaded.(fields{i}), 2) == 3
                                data.original_trajectory = loaded.(fields{i});
                                found = true;
                                break;
                            end
                        end
                        if ~found
                            error('无法找到有效的轨迹数据');
                        end
                    end
                case '.csv'
                    data.original_trajectory = readmatrix(fullpath);
                case '.txt'
                    data.original_trajectory = readmatrix(fullpath);
            end
            
            % 检查数据格式
            if size(data.original_trajectory, 2) ~= 3
                error('轨迹数据必须包含3列 (X, Y, Z)');
            end
            
            % 更新文件信息
            file_info.Text = ['当前文件: ' filename];
            
            % 显示原始轨迹
            plotTrajectory();
            
        catch ME
            uialert(fig, ['导入失败: ' ME.message], '错误', 'Icon', 'error');
        end
    end

    function processTrajectory(~, ~)
        if isempty(data.original_trajectory)
            uialert(fig, '请先导入轨迹数据！', '警告', 'Icon', 'warning');
            return;
        end
        
        try
            % 获取处理参数
            dt = params.interval.Value;
            max_vel = params.max_vel.Value;
            max_acc = params.max_acc.Value;
            smooth_factor = params.smooth.Value;
            
            % 计算原始轨迹段的向量和长度
            segments = diff(data.original_trajectory);
            segment_lengths = sqrt(sum(segments.^2, 2));
            total_length = sum(segment_lengths);
            
            % 根据采样方式确定采样点
            switch params.sampling.Value
                case '等时间间隔'
                    % 根据最大速度和时间间隔计算每段需要的点数
                    points_per_segment = ceil(segment_lengths / (max_vel * dt));
                    
                case '等弧长间隔'
                    % 根据指定的间隔计算每段需要的点数
                    points_per_segment = ceil(segment_lengths / dt);
            end
            
            % 预分配存储空间
            total_points = sum(points_per_segment) + 1;
            data.processed_trajectory = zeros(total_points, 3);
            data.processed_trajectory(1,:) = data.original_trajectory(1,:);
            
            % 在每段上进行采样
            current_point = 2;
            for i = 1:length(points_per_segment)
                % 当前段的起点和终点
                p1 = data.original_trajectory(i,:);
                p2 = data.original_trajectory(i+1,:);
                
                % 在当前段上均匀采样
                for j = 1:points_per_segment(i)
                    t = j / points_per_segment(i);
                    data.processed_trajectory(current_point,:) = p1 + t * (p2 - p1);
                    current_point = current_point + 1;
                end
            end
            
            % 确保最后一个点正确
            data.processed_trajectory(end,:) = data.original_trajectory(end,:);
            
            % 应用平滑（如果需要）
            if smooth_factor > 0
                window_size = max(3, round(size(data.processed_trajectory, 1) * smooth_factor / 10));
                if mod(window_size, 2) == 0
                    window_size = window_size + 1;
                end
                data.processed_trajectory = smoothdata(data.processed_trajectory, 'gaussian', window_size);
            end
            
            % 重新显示轨迹
            plotTrajectory();
            
        catch ME
            uialert(fig, ['处理失败: ' ME.message], '错误', 'Icon', 'error');
        end
    end

    function exportTrajectory(~, ~)
        if isempty(data.processed_trajectory)
            uialert(fig, '请先处理轨迹数据！', '警告', 'Icon', 'warning');
            return;
        end
        
        try
            [filename, pathname] = uiputfile({'*.mat', 'MAT文件 (*.mat)'; ...
                '*.csv', 'CSV文件 (*.csv)'; ...
                '*.txt', '文本文件 (*.txt)'});
            
            if isequal(filename, 0)
                return;
            end
            
            fullpath = fullfile(pathname, filename);
            [~, ~, ext] = fileparts(filename);
            
            switch lower(ext)
                case '.mat'
                    % 保存为与UR5轨迹设计器兼容的格式
                    trajectory_points = data.processed_trajectory;
                    save(fullpath, 'trajectory_points');
                case '.csv'
                    writematrix(data.processed_trajectory, fullpath);
                case '.txt'
                    writematrix(data.processed_trajectory, fullpath);
            end
            
            uialert(fig, '轨迹导出成功！', '成功', 'Icon', 'success');
            
        catch ME
            uialert(fig, ['导出失败: ' ME.message], '错误', 'Icon', 'error');
        end
    end
    
    function plotTrajectory()
        % 清除当前图形
        cla(ax_trajectory);
        hold(ax_trajectory, 'on');
        grid(ax_trajectory, 'on');
        
        % 设置视图
        view(ax_trajectory, 3);
        xlabel(ax_trajectory, 'X (m)');
        ylabel(ax_trajectory, 'Y (m)');
        zlabel(ax_trajectory, 'Z (m)');
        title(ax_trajectory, '轨迹可视化');
        
        % 获取所有轨迹点的范围
        all_points = data.original_trajectory;
        if ~isempty(data.processed_trajectory)
            all_points = [all_points; data.processed_trajectory];
        end
        
        % 设置坐标轴范围，添加一些边距
        margin = 0.1;
        min_vals = min(all_points) - margin;
        max_vals = max(all_points) + margin;
        xlim(ax_trajectory, [min_vals(1) max_vals(1)]);
        ylim(ax_trajectory, [min_vals(2) max_vals(2)]);
        zlim(ax_trajectory, [min_vals(3) max_vals(3)]);
        
        % 绘制原始轨迹（如果选中）
        if display_options.original_trajectory.Value
            plot3(ax_trajectory, data.original_trajectory(:,1), ...
                data.original_trajectory(:,2), ...
                data.original_trajectory(:,3), 'b-', 'LineWidth', 1.5);
        end
        
        % 绘制处理后的轨迹（如果存在且选中）
        if ~isempty(data.processed_trajectory) && display_options.processed_trajectory.Value
            plot3(ax_trajectory, data.processed_trajectory(:,1), ...
                data.processed_trajectory(:,2), ...
                data.processed_trajectory(:,3), 'r--', 'LineWidth', 1.5);
        end
        
        % 绘制采样点（如果选中）
        if display_options.sample_points.Value
            if ~isempty(data.processed_trajectory)
                plot3(ax_trajectory, data.processed_trajectory(:,1), ...
                    data.processed_trajectory(:,2), ...
                    data.processed_trajectory(:,3), 'r.', 'MarkerSize', 10);
            end
        end
        
        % 绘制关键点（如果选中）
        if display_options.key_points.Value
            plot3(ax_trajectory, data.original_trajectory(:,1), ...
                data.original_trajectory(:,2), ...
                data.original_trajectory(:,3), 'k.', 'MarkerSize', 12);
        end
        
        % 绘制速度向量（如果选中且存在处理后的轨迹）
        if display_options.velocity_vectors.Value && ~isempty(data.processed_trajectory)
            % 计算速度向量
            velocities = diff(data.processed_trajectory);
            % 每隔几个点绘制一个速度向量
            step = max(1, round(size(velocities, 1) / 20));
            for i = 1:step:size(velocities, 1)
                quiver3(ax_trajectory, data.processed_trajectory(i,1), ...
                    data.processed_trajectory(i,2), ...
                    data.processed_trajectory(i,3), ...
                    velocities(i,1), velocities(i,2), velocities(i,3), ...
                    1, 'g', 'LineWidth', 1);
            end
        end
        
        % 绘制加速度向量（如果选中且存在处理后的轨迹）
        if display_options.acceleration_vectors.Value && ~isempty(data.processed_trajectory)
            % 计算加速度向量
            accelerations = diff(diff(data.processed_trajectory));
            % 每隔几个点绘制一个加速度向量
            step = max(1, round(size(accelerations, 1) / 20));
            for i = 1:step:size(accelerations, 1)
                quiver3(ax_trajectory, data.processed_trajectory(i,1), ...
                    data.processed_trajectory(i,2), ...
                    data.processed_trajectory(i,3), ...
                    accelerations(i,1), accelerations(i,2), accelerations(i,3), ...
                    1, 'm', 'LineWidth', 1);
            end
        end
        
        % 绘制自定义点
        if ~isempty(data.custom_points)
            plot3(ax_trajectory, data.custom_points(:,1), ...
                data.custom_points(:,2), ...
                data.custom_points(:,3), 'c.', 'MarkerSize', 15);
        end
        
        % 保持图形比例
        axis(ax_trajectory, 'equal');
        
        % 更新图形
        drawnow;
    end

    function addCurrentPoint(~, ~)
        % 获取当前输入框中的坐标值
        try
            x = str2double(get(params.x_input, 'Value'));
            y = str2double(get(params.y_input, 'Value'));
            z = str2double(get(params.z_input, 'Value'));
            
            if isnan(x) || isnan(y) || isnan(z)
                uialert(fig, '请输入有效的坐标值！', '错误', 'Icon', 'error');
                return;
            end
            
            % 添加新点
            new_point = [x y z];
            if isempty(data.custom_points)
                data.custom_points = new_point;
            else
                data.custom_points = [data.custom_points; new_point];
            end
            
            % 更新列表显示
            updatePointsList();
            
            % 更新3D视图
            plotTrajectory();
            
        catch ME
            uialert(fig, ['添加点失败: ' ME.message], '错误', 'Icon', 'error');
        end
    end

    function deleteSelectedPoint(~, ~)
        try
            % 获取选中的点索引
            selected_idx = data.points_list.Value;
            if isempty(selected_idx)
                uialert(fig, '请先选择要删除的点！', '警告', 'Icon', 'warning');
                return;
            end
            
            % 删除选中的点
            data.custom_points(selected_idx, :) = [];
            
            % 更新列表显示
            updatePointsList();
            
            % 更新3D视图
            plotTrajectory();
            
        catch ME
            uialert(fig, ['删除点失败: ' ME.message], '错误', 'Icon', 'error');
        end
    end

    function updatePointsList()
        % 清空现有列表
        data.points_list.Items = {};
        
        % 如果没有点，直接返回
        if isempty(data.custom_points)
            return;
        end
        
        % 为每个点创建显示字符串
        point_strings = cell(size(data.custom_points, 1), 1);
        for i = 1:size(data.custom_points, 1)
            point_strings{i} = sprintf('点 %d: (%.2f, %.2f, %.2f)', ...
                i, data.custom_points(i,1), data.custom_points(i,2), data.custom_points(i,3));
        end
        
        % 更新列表显示
        data.points_list.Items = point_strings;
    end

    function pointSelected(~, ~)
        % 当选择点时的处理函数
        selected_idx = data.points_list.Value;
        if ~isempty(selected_idx)
            % 更新输入框显示选中点的坐标
            selected_point = data.custom_points(selected_idx, :);
            params.x_input.Value = num2str(selected_point(1));
            params.y_input.Value = num2str(selected_point(2));
            params.z_input.Value = num2str(selected_point(3));
        end
    end
end
