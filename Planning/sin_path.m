clear; close all; clc;

x_fixed = 250;
y_min = -120;
y_max = 120;
num = [30, 50, 100, 300];

for n = num
    y = linspace(y_min, y_max, n);
    z = 200 + 50 * sin(2 * pi * (y + 120) / 240);
    x = x_fixed * ones(size(y));
    path = [x', y', z'];
    filename = sprintf('sin_%d_points.txt', n);
    write_file = fopen(filename, 'wt');
    fprintf(write_file, '# 轨迹类型：正弦轨迹\n');
    fprintf(write_file, '# 点数：%d\n', n);
    fprintf(write_file, '# X(mm) Y(mm) Z(mm)\n');
    for i = 1:size(path, 1)
        fprintf(write_file, '%.2f %.2f %.2f\n', path(i,1), path(i,2), path(i,3));
    end
    fclose(write_file);
end
