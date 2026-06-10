%%第二笔

clc
clear

number=10;
% 定义两个点的坐标
point1 = [20, 90, 160];
point2 = [40, 90, 160]; % 假设的第二个点坐标

% 初始化角度矩阵
angleMatrix = zeros(number, 3); % 11个角度值，包括起始和结束点

% 线性插值计算中间点并计算角度
for i = 1:number
    % 计算插值参数
    t = (i-1)/(number-1);
    % 计算插值点坐标
    x = (1-t)*point1(1) + t*point2(1);
    y = (1-t)*point1(2) + t*point2(2);
    z = (1-t)*point1(3) + t*point2(3);
    
    % 调用函数计算角度
    [angle1, angle2, angle3] = calculateTrueAngles(x, y, z);
    
    % 将计算出的角度插入矩阵
    angleMatrix(i, :) = [angle1, angle2, angle3];
end

% 显示角度矩阵
disp(angleMatrix);

% 将角度矩阵输出为txt文件
filename = 'angleMatrix2.txt'; % 定义文件名
fileID = fopen(filename, 'w'); % 打开文件进行写入，'w'模式会在打开文件时清空文件

% 写入文件头部
fprintf(fileID, 'float angles[number][3] = {\n', number);

% 遍历矩阵并写入数据，从最后一行开始到第一行
for i = number:-1:1
    fprintf(fileID, '    {%.2f, %.2f, %.2f}', angleMatrix(i, 1), angleMatrix(i, 2), angleMatrix(i, 3));
    if i > 1
        fprintf(fileID, ',\n');
    else
        fprintf(fileID, '\n');
    end
end
% 写入文件尾部
fprintf(fileID, '};');

% 关闭文件
fclose(fileID);



function [angle1_ture, angle2_ture, angle3_ture] = calculateTrueAngles(x, y, z)
    % 常量定义
    L1 = 0;
    L2 = 105;
    L3 = 170;

    % 检查点是否在指定范围内
    if sqrt(x^2 + y^2 + z^2) > (L2 + L3)
        error('点位不对');
    else
        % 边长计算
        ab = L2;
        ac = L3;
        bc = sqrt(x^2 + y^2 + z^2);

        % 角度计算
        cbd = atan(z / sqrt(x^2 + y^2));  % 计算CBD角度
        cba = acos((bc^2 + ab^2 - ac^2) / (2 * bc * ab));  % 计算CBA角度
        cae = pi - acos((ab^2 + ac^2 - bc^2) / (2 * ab * ac));  % 计算CAE角度

        % 计算角度1, 2, 3
        angle1 = atan(x / y);  % 计算角度1
        angle2 = cbd + cba;  % 计算角度2
        angle3 = cae;  % 计算角度3

        % 将弧度转换为度
        angle1_deg = rad2deg(angle1);
        angle2_deg = rad2deg(angle2);
        angle3_deg = rad2deg(angle3);

        % 计算真实角度
        angle1_ture = 90 - angle1_deg / 1.5;
        angle2_ture = 120 - angle2_deg / 1.5;
        angle3_ture = 90 + angle3_deg / 1.5;

        % % 显示结果
        % fprintf('Angle 1 ture: %.2f degrees\n', angle1_ture);
        % fprintf('Angle 2 ture: %.2f degrees\n', angle2_ture);
        % fprintf('Angle 3 ture: %.2f degrees\n', angle3_ture);
    end
end