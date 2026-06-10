classdef TrajectoryGenerator < matlab.apps.AppBase
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        TrajectoryAxes      matlab.ui.control.UIAxes
        ShapeDropDown       matlab.ui.control.DropDown
        SizeSpinner         matlab.ui.control.Spinner
        PointsSpinner       matlab.ui.control.Spinner
        XOffsetSpinner      matlab.ui.control.Spinner
        YOffsetSpinner      matlab.ui.control.Spinner
        ZOffsetSpinner      matlab.ui.control.Spinner
        GenerateButton      matlab.ui.control.Button
        SaveButton          matlab.ui.control.Button
        PreviewButton       matlab.ui.control.Button
    end
    
    properties (Access = private)
        TrajectoryPoints = []  % 存储生成的轨迹点
    end
    
    methods (Access = private)
        function createComponents(app)
            % 创建UI界面
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 800 600];
            app.UIFigure.Name = '轨迹生成器';
            
            % 创建控制面板
            controlPanel = uipanel(app.UIFigure);
            controlPanel.Title = '轨迹参数';
            controlPanel.Position = [10 10 200 580];
            
            % 轨迹类型选择
            uilabel(controlPanel, 'Text', '轨迹类型:', 'Position', [10 520 80 22]);
            app.ShapeDropDown = uidropdown(controlPanel);
            app.ShapeDropDown.Items = {'圆形', '方形', '三角形', '螺旋线'};
            app.ShapeDropDown.Value = '圆形';
            app.ShapeDropDown.Position = [100 520 90 22];
            
            % 尺寸设置
            uilabel(controlPanel, 'Text', '尺寸 (m):', 'Position', [10 480 80 22]);
            app.SizeSpinner = uispinner(controlPanel);
            app.SizeSpinner.Limits = [0.1 0.5];
            app.SizeSpinner.Value = 0.2;
            app.SizeSpinner.Position = [100 480 90 22];
            
            % 采样点数
            uilabel(controlPanel, 'Text', '采样点数:', 'Position', [10 440 80 22]);
            app.PointsSpinner = uispinner(controlPanel);
            app.PointsSpinner.Limits = [10 100];
            app.PointsSpinner.Value = 50;
            app.PointsSpinner.Position = [100 440 90 22];
            
            % 位置偏移
            uilabel(controlPanel, 'Text', '位置偏移 (m):', 'Position', [10 400 80 22]);
            
            uilabel(controlPanel, 'Text', 'X:', 'Position', [10 370 20 22]);
            app.XOffsetSpinner = uispinner(controlPanel);
            app.XOffsetSpinner.Limits = [-1 1];
            app.XOffsetSpinner.Value = 0.5;
            app.XOffsetSpinner.Position = [30 370 60 22];
            
            uilabel(controlPanel, 'Text', 'Y:', 'Position', [100 370 20 22]);
            app.YOffsetSpinner = uispinner(controlPanel);
            app.YOffsetSpinner.Limits = [-1 1];
            app.YOffsetSpinner.Value = 0;
            app.YOffsetSpinner.Position = [120 370 60 22];
            
            uilabel(controlPanel, 'Text', 'Z:', 'Position', [10 340 20 22]);
            app.ZOffsetSpinner = uispinner(controlPanel);
            app.ZOffsetSpinner.Limits = [0 1.5];
            app.ZOffsetSpinner.Value = 0.5;
            app.ZOffsetSpinner.Position = [30 340 60 22];
            
            % 按钮
            app.GenerateButton = uibutton(controlPanel, 'push');
            app.GenerateButton.ButtonPushedFcn = @(~,~)generateTrajectory(app);
            app.GenerateButton.Text = '生成轨迹';
            app.GenerateButton.Position = [10 280 180 30];
            
            app.PreviewButton = uibutton(controlPanel, 'push');
            app.PreviewButton.ButtonPushedFcn = @(~,~)previewTrajectory(app);
            app.PreviewButton.Text = '预览轨迹';
            app.PreviewButton.Position = [10 240 180 30];
            
            app.SaveButton = uibutton(controlPanel, 'push');
            app.SaveButton.ButtonPushedFcn = @(~,~)saveTrajectory(app);
            app.SaveButton.Text = '保存轨迹';
            app.SaveButton.Enable = 'off';
            app.SaveButton.Position = [10 200 180 30];
            
            % 创建轨迹预览区域
            app.TrajectoryAxes = uiaxes(app.UIFigure);
            app.TrajectoryAxes.Position = [220 10 570 580];
            app.TrajectoryAxes.XLim = [-1 1];
            app.TrajectoryAxes.YLim = [-1 1];
            app.TrajectoryAxes.ZLim = [0 1.5];
            grid(app.TrajectoryAxes, 'on');
            view(app.TrajectoryAxes, 3);
            
            app.UIFigure.Visible = 'on';
        end
        
        function generateTrajectory(app)
            % 获取参数
            shape = app.ShapeDropDown.Value;
            size = app.SizeSpinner.Value;
            numPoints = app.PointsSpinner.Value;
            offset = [app.XOffsetSpinner.Value, ...
                     app.YOffsetSpinner.Value, ...
                     app.ZOffsetSpinner.Value];
            
            % 根据选择的形状生成轨迹
            switch shape
                case '圆形'
                    t = linspace(0, 2*pi, numPoints)';
                    x = size * cos(t) + offset(1);
                    y = size * sin(t) + offset(2);
                    z = ones(numPoints, 1) * offset(3);
                    
                case '方形'
                    points_per_side = round(numPoints/4);
                    t = linspace(0, 1, points_per_side)';
                    
                    % 生成四条边
                    x = []; y = []; z = [];
                    % 第一条边
                    x = [x; -size/2 + size*t];
                    y = [y; ones(points_per_side,1)*size/2];
                    % 第二条边
                    x = [x; ones(points_per_side,1)*size/2];
                    y = [y; size/2 - size*t];
                    % 第三条边
                    x = [x; size/2 - size*t];
                    y = [y; ones(points_per_side,1)*-size/2];
                    % 第四条边
                    x = [x; ones(points_per_side,1)*-size/2];
                    y = [y; -size/2 + size*t];
                    
                    x = x + offset(1);
                    y = y + offset(2);
                    z = ones(length(x), 1) * offset(3);
                    
                case '三角形'
                    points_per_side = round(numPoints/3);
                    t = linspace(0, 1, points_per_side)';
                    
                    % 计算三角形的顶点
                    vertices = size * [
                        -0.5, -0.289;  % 左下角
                         0.5, -0.289;  % 右下角
                         0.0,  0.577   % 顶点
                    ];
                    
                    % 生成三条边
                    x = []; y = [];
                    % 第一条边：左下到右下
                    x = [x; vertices(1,1) + (vertices(2,1)-vertices(1,1))*t];
                    y = [y; vertices(1,2) + (vertices(2,2)-vertices(1,2))*t];
                    % 第二条边：右下到顶点
                    x = [x; vertices(2,1) + (vertices(3,1)-vertices(2,1))*t];
                    y = [y; vertices(2,2) + (vertices(3,2)-vertices(2,2))*t];
                    % 第三条边：顶点到左下
                    x = [x; vertices(3,1) + (vertices(1,1)-vertices(3,1))*t];
                    y = [y; vertices(3,2) + (vertices(1,2)-vertices(3,2))*t];
                    
                    x = x + offset(1);
                    y = y + offset(2);
                    z = ones(length(x), 1) * offset(3);
                    
                case '螺旋线'
                    t = linspace(0, 4*pi, numPoints)';
                    x = size * cos(t) + offset(1);
                    y = size * sin(t) + offset(2);
                    z = linspace(offset(3)-size/2, offset(3)+size/2, numPoints)';
            end
            
            app.TrajectoryPoints = [x y z];
            app.SaveButton.Enable = 'on';
            
            % 预览轨迹
            previewTrajectory(app);
        end
        
        function previewTrajectory(app)
            if isempty(app.TrajectoryPoints)
                return;
            end
            
            cla(app.TrajectoryAxes);
            hold(app.TrajectoryAxes, 'on');
            
            % 绘制轨迹
            plot3(app.TrajectoryAxes, ...
                app.TrajectoryPoints(:,1), ...
                app.TrajectoryPoints(:,2), ...
                app.TrajectoryPoints(:,3), ...
                'b-', 'LineWidth', 2);
            
            % 绘制轨迹点
            scatter3(app.TrajectoryAxes, ...
                app.TrajectoryPoints(:,1), ...
                app.TrajectoryPoints(:,2), ...
                app.TrajectoryPoints(:,3), ...
                'r.');
            
            % 标记起点和终点
            plot3(app.TrajectoryAxes, ...
                app.TrajectoryPoints(1,1), ...
                app.TrajectoryPoints(1,2), ...
                app.TrajectoryPoints(1,3), ...
                'go', 'MarkerSize', 10, 'LineWidth', 2);
            
            plot3(app.TrajectoryAxes, ...
                app.TrajectoryPoints(end,1), ...
                app.TrajectoryPoints(end,2), ...
                app.TrajectoryPoints(end,3), ...
                'ro', 'MarkerSize', 10, 'LineWidth', 2);
            
            grid(app.TrajectoryAxes, 'on');
            xlabel(app.TrajectoryAxes, 'X (m)');
            ylabel(app.TrajectoryAxes, 'Y (m)');
            zlabel(app.TrajectoryAxes, 'Z (m)');
            view(app.TrajectoryAxes, 3);
            axis(app.TrajectoryAxes, 'equal');
            
            hold(app.TrajectoryAxes, 'off');
        end
        
        function saveTrajectory(app)
            if isempty(app.TrajectoryPoints)
                return;
            end
            
            % 打开保存对话框
            [filename, pathname] = uiputfile('*.mat', '保存轨迹数据');
            if isequal(filename,0) || isequal(pathname,0)
                return;
            end
            
            % 保存轨迹数据
            trajectoryData = struct();
            trajectoryData.points = app.TrajectoryPoints;
            trajectoryData.type = app.ShapeDropDown.Value;
            trajectoryData.size = app.SizeSpinner.Value;
            trajectoryData.offset = [app.XOffsetSpinner.Value, ...
                                   app.YOffsetSpinner.Value, ...
                                   app.ZOffsetSpinner.Value];
            
            save(fullfile(pathname, filename), 'trajectoryData');
            msgbox('轨迹数据已保存！', '成功');
        end
    end
    
    methods (Access = public)
        function app = TrajectoryGenerator
            createComponents(app)
            
            % Register the app with App Designer
            registerApp(app, app.UIFigure)
        end
    end
end 