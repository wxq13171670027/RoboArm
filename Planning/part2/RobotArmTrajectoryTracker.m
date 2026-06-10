classdef RobotArmTrajectoryTracker < matlab.apps.AppBase
    properties (Access = public)
        % UI组件
        UIFigure            matlab.ui.Figure
        RobotAxes          matlab.ui.control.UIAxes
        LoadTrajectoryButton    matlab.ui.control.Button
        CheckSingularityButton  matlab.ui.control.Button
        StartTrackingButton     matlab.ui.control.Button
        StopTrackingButton      matlab.ui.control.Button
        StatusLabel            matlab.ui.control.Label
        
        % 机器人相关组件
        Robot              % 机器人模型
        CurrentConfig      % 当前关节配置
        HomeConfig         % 初始配置
        LoadedTrajectory = []    % 存储导入的轨迹数据
        SingularPoints = []      % 存储检测到的奇异点
        IsAnimating = false      % 动画状态标志
        AnimationTimer           % 动画定时器
        TargetConfigs           % 目标关节配置序列
        TargetPath = []         % 目标路径存储
        ExecutorPath = []       % 存储执行过的路径点
        PathPoints = 100         % 路径点数量
        TotalTime = 5          % 总执行时间(s)
        VelocityProfile        % 速度曲线
        
        % 添加关节角度标签
        Joint1Label        matlab.ui.control.Label
        Joint2Label        matlab.ui.control.Label
        Joint3Label        matlab.ui.control.Label
        Joint4Label        matlab.ui.control.Label
        Joint5Label        matlab.ui.control.Label
        Joint6Label        matlab.ui.control.Label
        
        % 录制相关属性
        IsRecording = false    % 录制状态标志
        RecordedFrames = {}   % 存储录制的帧
    end
    
    methods (Access = private)
        % 导入轨迹
        function loadTrajectory(app)
            [filename, pathname] = uigetfile('*.mat', '选择轨迹数据文件');
            if isequal(filename, 0) || isequal(pathname, 0)
                return;
            end
            
            try
                % 加载轨迹数据
                data = load(fullfile(pathname, filename));
                if ~isfield(data, 'trajectoryData') || ~isfield(data.trajectoryData, 'points')
                    msgbox('无效的轨迹数据文件！', '错误', 'error');
                    return;
                end
                
                app.LoadedTrajectory = data.trajectoryData.points;
                app.plotTrajectory();
                
                app.StatusLabel.Text = '轨迹已加载，可以开始检查奇异点';
                app.CheckSingularityButton.Enable = 'on';
                
            catch ME
                msgbox(['加载轨迹失败: ' ME.message], '错误', 'error');
            end
        end
        
        % 检查奇异点
        function checkSingularity(app)
            if isempty(app.LoadedTrajectory)
                msgbox('请先加载轨迹！', '警告', 'warn');
                return;
            end
            
            app.StatusLabel.Text = '正在检查奇异点...';
            app.SingularPoints = [];
            
            % 创建逆运动学求解器
            ik = inverseKinematics('RigidBodyTree', app.Robot);
            weights = [1 1 1 1 1 1];
            
            % 对每个轨迹点进行检查
            for i = 1:size(app.LoadedTrajectory, 1)
                point = app.LoadedTrajectory(i,:);
                
                % 计算逆解
                target_transform = getTransform(app.Robot, app.CurrentConfig, 'tool0');
                target_transform(1:3,4) = point';
                [config, ~] = ik('tool0', target_transform, weights, app.CurrentConfig);
                
                % 计算雅可比矩阵
                J = geometricJacobian(app.Robot, config, 'tool0');
                
                % 检查是否为奇异点
                if rank(J) < 6 || cond(J) > 1e6
                    app.SingularPoints = [app.SingularPoints; i];
                end
            end
            
            % 更新状态显示
            if isempty(app.SingularPoints)
                app.StatusLabel.Text = '未检测到奇异点，可以开始轨迹跟踪';
            else
                app.StatusLabel.Text = sprintf('检测到 %d 个奇异点', length(app.SingularPoints));
            end
            
            app.StartTrackingButton.Enable = 'on';
            app.plotTrajectory();
        end
        
        % 开始跟踪
        function startTracking(app)
            if isempty(app.LoadedTrajectory)
                msgbox('请先加载轨迹！', '警告', 'warn');
                return;
            end
            
            try
                % 清空执行路径
                app.ExecutorPath = [];
                
                % 开始录制
                app.IsRecording = true;
                app.RecordedFrames = {};
                
                % 生成路径
                app.generatePath();
                
                % 禁用按钮
                app.LoadTrajectoryButton.Enable = 'off';
                app.CheckSingularityButton.Enable = 'off';
                app.StartTrackingButton.Enable = 'off';
                app.StopTrackingButton.Enable = 'on';
                
                % 设置定时器周期
                period = app.TotalTime / app.PathPoints;
                
                % 开始动画
                app.IsAnimating = true;
                app.AnimationTimer = timer(...
                    'ExecutionMode', 'fixedRate', ...
                    'Period', period, ...
                    'TimerFcn', @(~,~)app.updateAnimation(), ...
                    'ErrorFcn', @(~,~)app.stopTracking(), ...
                    'StopFcn', @(~,~)app.stopTracking());
                
                start(app.AnimationTimer);
                app.StatusLabel.Text = '正在执行轨迹...';
            catch ME
                app.stopTracking();
                msgbox(['开始跟踪失败: ' ME.message], '错误', 'error');
            end
        end
        
        % 停止跟踪
        function stopTracking(app)
            if ~isempty(app.AnimationTimer) && isvalid(app.AnimationTimer)
                stop(app.AnimationTimer);
                delete(app.AnimationTimer);
            end
            
            app.IsAnimating = false;
            
            % 如果有录制的帧，自动保存
            if app.IsRecording && ~isempty(app.RecordedFrames)
                app.IsRecording = false;
                
                % 自动生成文件名（使用时间戳）
                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                filename = ['RobotAnimation_' timestamp '.gif'];
                
                try
                    % 创建GIF文件
                    for i = 1:length(app.RecordedFrames)
                        % 将帧转换为索引图像
                        frame = app.RecordedFrames{i};
                        if ~isempty(frame)
                            % 确保frame.cdata是uint8类型
                            if ~isa(frame.cdata, 'uint8')
                                frame.cdata = uint8(frame.cdata);
                            end
                            [imind, cm] = rgb2ind(frame.cdata, 256);
                            
                            % 写入GIF文件
                            if i == 1
                                imwrite(imind, cm, filename, 'gif', ...
                                    'Loopcount', inf, ...
                                    'DelayTime', app.TotalTime/length(app.RecordedFrames));
                            else
                                imwrite(imind, cm, filename, 'gif', ...
                                    'WriteMode', 'append', ...
                                    'DelayTime', app.TotalTime/length(app.RecordedFrames));
                            end
                        end
                    end
                    app.StatusLabel.Text = ['动画已保存为: ' filename];
                    msgbox(['动画已保存为: ' filename], '保存成功');
                catch ME
                    msgbox(['保存动画失败: ' ME.message], '错误', 'error');
                    disp(['错误详情: ' getReport(ME)]);  % 输出详细错误信息
                end
            end
            
            % 恢复按钮状态
            app.LoadTrajectoryButton.Enable = 'on';
            app.CheckSingularityButton.Enable = 'on';
            app.StartTrackingButton.Enable = 'on';
            app.StopTrackingButton.Enable = 'off';
            
            app.StatusLabel.Text = '轨迹执行已停止';
        end
        
        % 修改生成路径方法
        function generatePath(app)
            try
                % 使用更多的插值点
                num_points = app.PathPoints;
                
                % 对加载的轨迹进行插值
                t = linspace(0, 1, size(app.LoadedTrajectory, 1));
                t_new = linspace(0, 1, num_points);
                
                % 对 x,y,z 分别进行插值
                x = interp1(t, app.LoadedTrajectory(:,1), t_new, 'spline');
                y = interp1(t, app.LoadedTrajectory(:,2), t_new, 'spline');
                z = interp1(t, app.LoadedTrajectory(:,3), t_new, 'spline');
                
                % 初始化数组
                app.TargetConfigs = zeros(num_points, 6);
                app.TargetPath = [x; y; z];
                app.TargetConfigs(1,:) = app.CurrentConfig;
                
                % 创建逆运动学求解器
                ik = inverseKinematics('RigidBodyTree', app.Robot);
                ik.SolverParameters.AllowRandomRestart = true;
                weights = [1 1 1 1 1 1];
                
                % 获取初始位姿
                current_config = app.CurrentConfig;
                
                fprintf('\n开始生成路径...\n');
                
                % 对每个插值点求解逆运动学
                for i = 1:num_points
                    target_transform = getTransform(app.Robot, current_config, 'tool0');
                    target_transform(1:3,4) = app.TargetPath(:,i);
                    
                    try
                        [config, info] = ik('tool0', target_transform, weights, current_config);
                        if info.ExitFlag > 0 && ~any(isnan(config)) && ~any(isinf(config))
                            app.TargetConfigs(i,:) = config;
                            current_config = config;
                            fprintf('  ✓ 点 %d: 成功求解\n', i);
                        else
                            error('逆解失败');
                        end
                    catch ME
                        fprintf('  ! 点 %d: 求解失败，使用上一个配置\n', i);
                        if i > 1
                            app.TargetConfigs(i,:) = app.TargetConfigs(i-1,:);
                        end
                    end
                end
                
                % 生成速度曲线
                app.VelocityProfile = ones(1, num_points);  % 使用恒定速度
                
                fprintf('\n路径生成完成！\n');
                
            catch ME
                app.TargetConfigs = [];
                app.stopTracking();
                msgbox(['路径生成失败: ' ME.message], '错误', 'error');
                rethrow(ME);
            end
        end
        
        % 修改更新动画方法
        function updateAnimation(app)
            if isempty(app.TargetConfigs)
                app.stopTracking();
                return;
            end
            
            try
                % 直接使用目标配置
                app.CurrentConfig = app.TargetConfigs(1,:);
                
                % 验证配置
                if any(isnan(app.CurrentConfig)) || any(isinf(app.CurrentConfig))
                    error('无效的关节配置');
                end
                
                % 更新显示
                app.plotRobot();
                app.updateJointLabels();
                
                % 获取当前末端执行器位置
                transform = getTransform(app.Robot, app.CurrentConfig, 'tool0');
                current_pos = transform(1:3,4);
                
                % 绘制实际轨迹
                hold(app.RobotAxes, 'on');
                plot3(app.RobotAxes, current_pos(1), current_pos(2), current_pos(3), ...
                    '.', 'Color', [0 0.4470 0.7410], 'MarkerSize', 10, ...
                    'Tag', 'ExecutorPath');
                hold(app.RobotAxes, 'off');
                
                % 如果正在录制，捕获当前帧
                if app.IsRecording
                    try
                        frame = getframe(app.RobotAxes);
                        if ~isempty(frame.cdata)
                            app.RecordedFrames{end+1} = frame;
                        end
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                    end
                end
                
                % 移除已执行的配置
                app.TargetConfigs(1,:) = [];
                
                % 更新状态显示
                progress = (app.PathPoints - size(app.TargetConfigs,1)) / app.PathPoints * 100;
                app.StatusLabel.Text = sprintf('正在执行轨迹... %.1f%%', progress);
                
                drawnow;
                
            catch ME
                app.stopTracking();
                msgbox(['动画更新失败: ' ME.message], '错误', 'error');
            end
        end
        
        % 绘制轨迹
        function plotTrajectory(app)
            % 绘制机器人
            app.plotRobot();
            
            % 绘制轨迹
            if ~isempty(app.LoadedTrajectory)
                hold(app.RobotAxes, 'on');
                
                % 绘制目标轨迹线
                h = plot3(app.RobotAxes, ...
                    app.LoadedTrajectory(:,1), ...
                    app.LoadedTrajectory(:,2), ...
                    app.LoadedTrajectory(:,3), ...
                    'g-', 'LineWidth', 2);
                h.Tag = 'PathLine';
                
                % 如果有实际轨迹，也绘制出来
                if ~isempty(app.TargetPath)
                    h = plot3(app.RobotAxes, ...
                        app.TargetPath(1,:), ...
                        app.TargetPath(2,:), ...
                        app.TargetPath(3,:), ...
                        'b--', 'LineWidth', 1.5);
                    h.Tag = 'ExecutorPath';
                end
                
                % 标记奇异点
                if ~isempty(app.SingularPoints)
                    points = app.LoadedTrajectory(app.SingularPoints, :);
                    h = scatter3(app.RobotAxes, points(:,1), points(:,2), points(:,3), ...
                        100, 'r*', 'LineWidth', 2);
                    h.Tag = 'PathPoints';
                end
                
                hold(app.RobotAxes, 'off');
            end
        end
        
        % 绘制机器人
        function plotRobot(app)
            % 保存当前警告状态
            oldWarningState = warning;
            warning('off', 'all');
            
            try
                % 清除当前轴，但保留特定标签的对象
                h = findobj(app.RobotAxes);
                for i = 1:length(h)
                    try
                        if isvalid(h(i))  % 检查对象是否有效
                            tag = get(h(i), 'Tag');
                            if ~isempty(tag) && ~any(strcmp(tag, {'PathPoints', 'PointLabel', 'PathLine', 'ExecutorPath'}))
                                delete(h(i));
                            end
                        end
                    catch
                        % 忽略无效对象
                        continue;
                    end
                end
                
                % 创建一个临时图形窗口来获取机器人的可视化数据
                tempFig = figure('Visible', 'off');
                tempAx = axes(tempFig);
                
                % 在临时轴上显示机器人
                show(app.Robot, app.CurrentConfig, 'Parent', tempAx);
                
                % 获取机器人图形对象
                robotObjects = findobj(tempAx);
                
                % 将机器人图形对象复制到主界面
                for i = 1:length(robotObjects)
                    if isvalid(robotObjects(i)) && ~strcmp(get(robotObjects(i), 'Type'), 'axes')
                        copyobj(robotObjects(i), app.RobotAxes);
                    end
                end
                
                % 删除临时图形窗口
                delete(tempFig);
                
                % 设置视图属性
                view(app.RobotAxes, 3);
                grid(app.RobotAxes, 'on');
                xlabel(app.RobotAxes, 'X (m)');
                ylabel(app.RobotAxes, 'Y (m)');
                zlabel(app.RobotAxes, 'Z (m)');
                xlim(app.RobotAxes, [-1 1]);
                ylim(app.RobotAxes, [-1 1]);
                zlim(app.RobotAxes, [0 1.5]);
                
            catch ME
                warning(oldWarningState);
                rethrow(ME);
            end
            
            % 恢复警告状态
            warning(oldWarningState);
        end
        
        % 更新关节角度显示
        function updateJointLabels(app)
            % 将弧度转换为角度
            angles = rad2deg(app.CurrentConfig);
            
            % 更新标签文本
            app.Joint1Label.Text = sprintf('Joint 1: %.1f°', angles(1));
            app.Joint2Label.Text = sprintf('Joint 2: %.1f°', angles(2));
            app.Joint3Label.Text = sprintf('Joint 3: %.1f°', angles(3));
            app.Joint4Label.Text = sprintf('Joint 4: %.1f°', angles(4));
            app.Joint5Label.Text = sprintf('Joint 5: %.1f°', angles(5));
            app.Joint6Label.Text = sprintf('Joint 6: %.1f°', angles(6));
        end
    end
    
    methods (Access = public)
        % 构造函数
        function app = RobotArmTrajectoryTracker
            % 创建UI界面
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1200 800];
            app.UIFigure.Name = '机器人轨迹跟踪器';
            
            % 创建机器人显示区域
            app.RobotAxes = uiaxes(app.UIFigure);
            app.RobotAxes.Position = [320 200 870 590];
            
            % 创建控制面板
            trajectoryPanel = uipanel(app.UIFigure);
            trajectoryPanel.Title = '轨迹控制';
            trajectoryPanel.Position = [320 10 870 180];
            
            % 添加按钮
            app.LoadTrajectoryButton = uibutton(trajectoryPanel, 'push');
            app.LoadTrajectoryButton.ButtonPushedFcn = @(~,~)loadTrajectory(app);
            app.LoadTrajectoryButton.Position = [10 120 120 30];
            app.LoadTrajectoryButton.Text = '加载轨迹';
            
            app.CheckSingularityButton = uibutton(trajectoryPanel, 'push');
            app.CheckSingularityButton.ButtonPushedFcn = @(~,~)checkSingularity(app);
            app.CheckSingularityButton.Position = [140 120 120 30];
            app.CheckSingularityButton.Text = '检查奇异点';
            app.CheckSingularityButton.Enable = 'off';
            
            app.StartTrackingButton = uibutton(trajectoryPanel, 'push');
            app.StartTrackingButton.ButtonPushedFcn = @(~,~)startTracking(app);
            app.StartTrackingButton.Position = [270 120 120 30];
            app.StartTrackingButton.Text = '开始跟踪';
            app.StartTrackingButton.Enable = 'off';
            
            app.StopTrackingButton = uibutton(trajectoryPanel, 'push');
            app.StopTrackingButton.ButtonPushedFcn = @(~,~)stopTracking(app);
            app.StopTrackingButton.Position = [400 120 120 30];
            app.StopTrackingButton.Text = '停止跟踪';
            app.StopTrackingButton.Enable = 'off';
            
            % 添加状态标签
            app.StatusLabel = uilabel(trajectoryPanel);
            app.StatusLabel.Position = [10 80 850 30];
            app.StatusLabel.Text = '请加载轨迹数据';
            
            % 初始化机器人
            app.Robot = loadrobot('universalUR5', 'DataFormat', 'row');
            app.HomeConfig = homeConfiguration(app.Robot);
            
            % 设置初始关节角度（与RobotArmGUI相同）
            app.CurrentConfig = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
            
            % 创建左侧控制面板
            controlPanel = uipanel(app.UIFigure);
            controlPanel.Title = '关节角度控制';
            controlPanel.Position = [10 10 300 780];
            
            % 添加关节角度标签
            labelHeight = 30;
            labelSpacing = 40;
            labelWidth = 280;  % 添加标签宽度
            startY = 720;  % 从顶部开始
            
            app.Joint1Label = uilabel(controlPanel);
            app.Joint1Label.Position = [10 startY labelWidth labelHeight];
            app.Joint1Label.Text = 'Joint 1: 0°';
            
            app.Joint2Label = uilabel(controlPanel);
            app.Joint2Label.Position = [10 startY-labelSpacing labelWidth labelHeight];
            app.Joint2Label.Text = 'Joint 2: -90°';
            
            app.Joint3Label = uilabel(controlPanel);
            app.Joint3Label.Position = [10 startY-2*labelSpacing labelWidth labelHeight];
            app.Joint3Label.Text = 'Joint 3: 90°';
            
            app.Joint4Label = uilabel(controlPanel);
            app.Joint4Label.Position = [10 startY-3*labelSpacing labelWidth labelHeight];
            app.Joint4Label.Text = 'Joint 4: -90°';
            
            app.Joint5Label = uilabel(controlPanel);
            app.Joint5Label.Position = [10 startY-4*labelSpacing labelWidth labelHeight];
            app.Joint5Label.Text = 'Joint 5: -90°';
            
            app.Joint6Label = uilabel(controlPanel);
            app.Joint6Label.Position = [10 startY-5*labelSpacing labelWidth labelHeight];
            app.Joint6Label.Text = 'Joint 6: 0°';
            
            % 显示界面
            app.UIFigure.Visible = 'on';
            app.plotRobot();
        end
    end
end 