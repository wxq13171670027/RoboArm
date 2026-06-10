clc; clearvars; close all
l1 = 0.2; l2 = 0.15; % 后臂和前臂的长度

% 初始化串口
serialObj = serialport("COM21", 115200);
configureTerminator(serialObj, "LF");  % 设置行终止符为换行
flush(serialObj);  % 清空缓冲区

pur_pos = [0.1, 0.2;0.2, 0.1];
n = 1;

while n <= 2
    x = pur_pos(n, 1);
    y = pur_pos(n, 2);
    
    r = sqrt(x^2 + y^2);
    a = (r^2 + l1^2 - l2^2) / (2 * r * l1);
    
    if abs(a) <= 1
        b = sqrt(1 - a^2);
        c = x / r;
        s = y / r;
        c1 = c * a + s * b;
        s1 = s * a - c * b;
        theta1 = atan2(s1, c1);
        
        ax = l1 * c1;
        ay = l1 * s1;
        bx = x - ax;
        by = y - ay;
        c2 = (ax * bx + ay * by) / (l1 * l2);
        s2 = sqrt(1 - c2^2);
        theta2 = atan2(s2, c2);
        
        q1 = theta1 * 180 / pi;
        q2 = theta2 * 180 / pi;
        
        disp([ num2str(q1), ',',num2str(q2)]);
        
        % 发送坐标给Arduino
        command = [num2str(q1), ',', num2str(q2)];
        writeline(serialObj, command);
        %disp('已发送指令，等待机械臂移动...');
        
        % 等待Arduino返回完成信号
        response = "";
        while isempty(response) || ~contains(response, "DONE")
            if serialObj.NumBytesAvailable > 0
                response = readline(serialObj);
                disp(['收到响应: ', response]);
            end
            pause(0.1);  % 小延迟，避免CPU过载
        end
        
        disp('机械臂已到达目标位置');
    else
        disp(['目标点 ', num2str(n), ' 不可达']);
    end
    
    n = n + 1;
end

% 清理
clear serialObj;
disp('程序结束');