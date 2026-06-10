function trajectory_processor
    % 创建主窗口
    fig = uifigure('Name', '轨迹处理工具', 'Position', [100 100 1400 800]);
    
    % 创建网格布局
    grid = uigridlayout(fig, [1 3]);
    grid.ColumnWidth = {'1x', '1x', '1x'};
    
    % 创建三个面板
    left_panel = uipanel(grid, 'Title', '轨迹导入/导出');
    middle_panel = uipanel(grid, 'Title', '轨迹处理参数');
    right_panel = uipanel(grid, 'Title', '可视化');
    
    % === 左侧面板：轨迹导入/导出 ===
    left_layout = uigridlayout(left_panel, [8 1]);
    left_layout.RowHeight = {'fit', 'fit', 'fit', 'fit', 'fit', '1x', 'fit', 'fit'};
    
    % 导入按钮
    import_btn = uibutton(left_layout, 'Text', '导入轨迹文件', ...
        'ButtonPushedFcn', @importTrajectory);
    
    % 显示当前文件
    file_label = uilabel(left_layout, 'Text', '当前文件: 无');
    
    % 轨迹信息
    info_label = uilabel(left_layout, 'Text', '轨迹信息: ');
    
    % 导出选项组
    export_group = uibuttongroup(left_layout, 'Title', '导出格式');
    export_group.Layout.Row = 4;
    
    % 创建导出格式选项
    formats = {'CSV文件 (.csv)', 'MAT文件 (.mat)', '文本文件 (.txt)'};
    radio_btns = gobjects(length(formats), 1);
    for i = 1:length(formats)
        radio_btns(i) = uiradiobutton(export_group, ...
            'Text', formats{i}, ...
            'Position', [10, 110-30*i, 150, 22]);
    end
    
    % 导出设置
    export_settings = uipanel(left_layout, 'Title', '导出设置');
    export_settings_layout = uigridlayout(export_settings, [3 2]);
    export_settings_layout.RowHeight = {'fit', 'fit', 'fit'};
    export_settings_layout.ColumnWidth = {'fit', '1x'};
    
    % 添加导出设置选项
    uilabel(export_settings_layout, 'Text', '时间戳');
    add_timestamp = uicheckbox(export_settings_layout, 'Value', true);
    
    uilabel(export_settings_layout, 'Text', '关节角度');
    add_joints = uicheckbox(export_settings_layout, 'Value', true);
    
    uilabel(export_settings_layout, 'Text', '末端位姿');
    add_pose = uicheckbox(export_settings_layout, 'Value', true);
    
    % 导出按钮
    export_btn = uibutton(left_layout, 'Text', '导出轨迹', ...
        'ButtonPushedFcn', @exportTrajectory);
    
    % === 中间面板：轨迹处理参数 ===
    middle_layout = uigridlayout(middle_panel, [12 2]);
    middle_layout.ColumnWidth = {'fit', '1x'};
    middle_layout.RowHeight = repmat({'fit'}, 1, 12);
    
    % 采样方式
    uilabel(middle_layout, 'Text', '采样方式:');
    sampling_method = uidropdown(middle_layout, ...
        'Items', {'等时间间隔', '等距离间隔', '关键点采样'});
    
    % 采样间隔
    uilabel(middle_layout, 'Text', '采样间隔:');
    sampling_interval = uispinner(middle_layout, ...
        'Value', 0.01, 'Limits', [0.001 1], 'Step', 0.001);
    
    % 插值方法
    uilabel(middle_layout, 'Text', '插值方法:');
    interp_method = uidropdown(middle_layout, ...
        'Items', {'线性插值', '三次样条插值', 'B样条插值'});
    
    % 平滑参数
    uilabel(middle_layout, 'Text', '平滑参数:');
    smooth_param = uispinner(middle_layout, ...
        'Value', 0.5, 'Limits', [0 1], 'Step', 0.1);
    
    % 速度限制
    uilabel(middle_layout, 'Text', '最大速度 (m/s):');
    max_velocity = uispinner(middle_layout, ...
        'Value', 1.0, 'Limits', [0.1 2], 'Step', 0.1);
    
    % 加速度限制
    uilabel(middle_layout, 'Text', '最大加速度 (m/s²):');
    max_accel = uispinner(middle_layout, ...
        'Value', 2.0, 'Limits', [0.1 5], 'Step', 0.1);
    
    % 处理按钮
    process_btn = uibutton(middle_layout, 'Text', '处理轨迹', ...
        'ButtonPushedFcn', @processTrajectory);
    process_btn.Layout.Column = [1 2];
    
    % === 右侧面板：可视化 ===
    right_layout = uigridlayout(right_panel, [2 1]);
    right_layout.RowHeight = {'4x', '1x'};
    
    % 创建绘图区域
    plot_panel = uipanel(right_layout);
    ax = uiaxes(plot_panel);
    ax.Position = [10 10 plot_panel.Position(3)-20 plot_panel.Position(4)-20];
    ax.Title.String = '轨迹可视化';
    ax.XLabel.String = 'X (m)';
    ax.YLabel.String = 'Y (m)';
    ax.ZLabel.String = 'Z (m)';
    ax.View = [30 30];
    ax.Box = 'on';
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    ax.ZGrid = 'on';
    
    % 创建图例面板
    legend_panel = uipanel(right_layout, 'Title', '图例');
    legend_layout = uigridlayout(legend_panel, [2 3]);
    
    % 添加图例控件
    uicheckbox(legend_layout, 'Text', '原始轨迹', 'Value', true);
    uicheckbox(legend_layout, 'Text', '处理后轨迹', 'Value', true);
    uicheckbox(legend_layout, 'Text', '采样点', 'Value', true);
    uicheckbox(legend_layout, 'Text', '速度向量', 'Value', false);
    uicheckbox(legend_layout, 'Text', '加速度向量', 'Value', false);
    uicheckbox(legend_layout, 'Text', '关键点', 'Value', true);
    
    % 存储数据
    data = struct();
    data.original_trajectory = [];
    data.processed_trajectory = [];
    data.joint_angles = [];
    data.timestamps = [];
    
    % === 回调函数 ===
    function importTrajectory(~, ~)
        [filename, pathname] = uigetfile({'*.mat;*.csv;*.txt', '轨迹文件 (*.mat, *.csv, *.txt)'});
        if filename ~= 0
            try
                [~, ~, ext] = fileparts(filename);
                switch ext
                    case '.mat'
                        loaded_data = load(fullfile(pathname, filename));
                        data.original_trajectory = loaded_data.trajectory_points;
                        if isfield(loaded_data, 'joint_configs')
                            data.joint_angles = loaded_data.joint_configs;
                        end
                    case '.csv'
                        data.original_trajectory = readmatrix(fullfile(pathname, filename));
                    case '.txt'
                        data.original_trajectory = readmatrix(fullfile(pathname, filename));
                end
                
                % 更新UI
                file_label.Text = ['当前文件: ' filename];
                info_label.Text = sprintf('轨迹信息: %d 个点', size(data.original_trajectory, 1));
                
                % 绘制轨迹
                cla(ax);
                hold(ax, 'on');
                plot3(ax, data.original_trajectory(:,1), ...
                    data.original_trajectory(:,2), ...
                    data.original_trajectory(:,3), 'b-', 'LineWidth', 2);
                scatter3(ax, data.original_trajectory(:,1), ...
                    data.original_trajectory(:,2), ...
                    data.original_trajectory(:,3), 'r.');
                hold(ax, 'off');
                
            catch e
                uialert(fig, ['导入错误: ' e.message], '错误', 'Icon', 'error');
            end
        end
    end
    
    function processTrajectory(~, ~)
        if isempty(data.original_trajectory)
            uialert(fig, '请先导入轨迹数据！', '警告', 'Icon', 'warning');
            return;
        end
        
        try
            % 根据选择的方法处理轨迹
            switch sampling_method.Value
                case '等时间间隔'
                    data.processed_trajectory = resampleTrajectoryTime(data.original_trajectory, ...
                        sampling_interval.Value);
                case '等距离间隔'
                    data.processed_trajectory = resampleTrajectoryDistance(data.original_trajectory, ...
                        sampling_interval.Value);
                case '关键点采样'
                    data.processed_trajectory = sampleKeyPoints(data.original_trajectory, ...
                        sampling_interval.Value);
            end
            
            % 应用插值
            switch interp_method.Value
                case '线性插值'
                    data.processed_trajectory = linearInterpolation(data.processed_trajectory);
                case '三次样条插值'
                    data.processed_trajectory = splineInterpolation(data.processed_trajectory, ...
                        smooth_param.Value);
                case 'B样条插值'
                    data.processed_trajectory = bsplineInterpolation(data.processed_trajectory, ...
                        smooth_param.Value);
            end
            
            % 应用速度和加速度限制
            data.processed_trajectory = applyKinematicConstraints(data.processed_trajectory, ...
                max_velocity.Value, max_accel.Value);
            
            % 更新显示
            cla(ax);
            hold(ax, 'on');
            plot3(ax, data.original_trajectory(:,1), ...
                data.original_trajectory(:,2), ...
                data.original_trajectory(:,3), 'b--', 'LineWidth', 1);
            plot3(ax, data.processed_trajectory(:,1), ...
                data.processed_trajectory(:,2), ...
                data.processed_trajectory(:,3), 'r-', 'LineWidth', 2);
            scatter3(ax, data.processed_trajectory(:,1), ...
                data.processed_trajectory(:,2), ...
                data.processed_trajectory(:,3), 'k.');
            hold(ax, 'off');
            
            legend(ax, '原始轨迹', '处理后轨迹', '采样点');
            
        catch e
            uialert(fig, ['处理错误: ' e.message], '错误', 'Icon', 'error');
        end
    end
    
    function exportTrajectory(~, ~)
        if isempty(data.processed_trajectory)
            uialert(fig, '请先处理轨迹数据！', '警告', 'Icon', 'warning');
            return;
        end
        
        try
            % 获取选中的导出格式
            selected_format = '';
            for i = 1:length(radio_btns)
                if radio_btns(i).Value
                    selected_format = radio_btns(i).Text;
                    break;
                end
            end
            
            % 准备导出数据
            export_data = data.processed_trajectory;
            if add_timestamp.Value
                timestamps = (0:size(export_data,1)-1)' * sampling_interval.Value;
                export_data = [timestamps, export_data];
            end
            
            if add_joints.Value && ~isempty(data.joint_angles)
                export_data = [export_data, cell2mat(data.joint_angles)];
            end
            
            % 导出文件
            [filename, pathname] = uiputfile({'*.csv', 'CSV文件 (*.csv)'; ...
                '*.mat', 'MAT文件 (*.mat)'; ...
                '*.txt', '文本文件 (*.txt)'}, ...
                '保存轨迹');
            
            if filename ~= 0
                filepath = fullfile(pathname, filename);
                [~, ~, ext] = fileparts(filepath);
                
                switch ext
                    case '.mat'
                        save(filepath, 'export_data');
                    case '.csv'
                        writematrix(export_data, filepath);
                    case '.txt'
                        writematrix(export_data, filepath);
                end
                
                uialert(fig, '轨迹已成功导出！', '成功', 'Icon', 'success');
            end
            
        catch e
            uialert(fig, ['导出错误: ' e.message], '错误', 'Icon', 'error');
        end
    end
end

% === 辅助函数 ===
function trajectory = resampleTrajectoryTime(trajectory, dt)
    % 等时间间隔重采样
    t = 0:dt:1;
    n = size(trajectory, 1);
    t_old = linspace(0, 1, n);
    trajectory = interp1(t_old, trajectory, t', 'spline');
end

function trajectory = resampleTrajectoryDistance(trajectory, ds)
    % 等距离间隔重采样
    distances = [0; cumsum(sqrt(sum(diff(trajectory).^2, 2)))];
    total_distance = distances(end);
    new_distances = (0:ds:total_distance)';
    trajectory = interp1(distances, trajectory, new_distances, 'spline');
end

function trajectory = sampleKeyPoints(trajectory, threshold)
    % 关键点采样
    dists = sqrt(sum(diff(trajectory).^2, 2));
    key_points = [1; find(dists > threshold); size(trajectory,1)];
    trajectory = trajectory(key_points, :);
end

function trajectory = linearInterpolation(trajectory)
    % 线性插值
    t = linspace(0, 1, size(trajectory,1))';
    t_new = linspace(0, 1, 2*size(trajectory,1))';
    trajectory = interp1(t, trajectory, t_new, 'linear');
end

function trajectory = splineInterpolation(trajectory, smooth_param)
    % 三次样条插值
    t = linspace(0, 1, size(trajectory,1))';
    t_new = linspace(0, 1, 2*size(trajectory,1))';
    trajectory = csaps(t, trajectory', smooth_param, t_new)';
end

function trajectory = bsplineInterpolation(trajectory, smooth_param)
    % B样条插值
    t = linspace(0, 1, size(trajectory,1))';
    t_new = linspace(0, 1, 2*size(trajectory,1))';
    
    % 使用简单的B样条实现
    trajectory = splineInterpolation(trajectory, smooth_param);
end

function trajectory = applyKinematicConstraints(trajectory, max_vel, max_acc)
    % 应用运动学约束
    dt = 0.01;  % 时间步长
    
    % 计算速度和加速度
    velocities = diff(trajectory) / dt;
    accelerations = diff(velocities) / dt;
    
    % 限制速度
    vel_magnitudes = sqrt(sum(velocities.^2, 2));
    scale_factors = ones(size(vel_magnitudes));
    over_speed = vel_magnitudes > max_vel;
    scale_factors(over_speed) = max_vel ./ vel_magnitudes(over_speed);
    velocities = velocities .* scale_factors;
    
    % 限制加速度
    acc_magnitudes = sqrt(sum(accelerations.^2, 2));
    scale_factors = ones(size(acc_magnitudes));
    over_acc = acc_magnitudes > max_acc;
    scale_factors(over_acc) = max_acc ./ acc_magnitudes(over_acc);
    accelerations = accelerations .* scale_factors;
    
    % 重建轨迹
    trajectory = cumsum([trajectory(1,:); velocities * dt], 1);
end
