clc; clearvars; close all
 l1 = 0.2; l2 = 0.15; % 后臂和前臂的长度
% serialportlist("available")
 serialObj = serialport("COM21",115200); % 连接后自动reset下位机
 %%
 
pur_pos = [888,888;0.1, 0.2;0.15,0.1;0.2,0.05;0.1,0.07; 0.1, 0.1;999,999];
n=2;
while n <= 7
     x = pur_pos(n, 1);
     y = pur_pos(n, 2);
     if n==7
         q1 = 999;
         q2 = 999;
         disp([q1 q2])
         writeline(serialObj, [num2str(q1),',',num2str(q2)]);
         break
     end

     r = sqrt(x^2+y^2);
     a = (r^2+l1^2-l2^2)/2/r/l1;
     if a<=1
         b=sqrt(1-a^2);
         c = x/r; s = y/r;
         c1 = c*a+s*b; s1 = s*a-c*b;
         theta1 = atan2(s1,c1);
         ax = l1*c1; ay = l1*s1;
         bx = x-ax; by = y - ay;
         c2 = (ax*bx+ay*by)/l1/l2;
         s2 = sqrt(1-c2^2);
         theta2 = atan2(s2,c2);
         q1 = theta1*180/pi;
         q2 = theta2*180/pi;
         disp([q1 q2])
         writeline(serialObj, [num2str(q1),',',num2str(q2)]);
     end
     n=n+1;
   
end
%%

clear serialObj;
%{
userInput = ''; % 初始化用户输入

while ~strcmpi(userInput, 'end') % 循环直到用户输入'end'（不区分大小写）
    userInput = input('请输入指令 (输入"end"以清空串口并退出): ', 's'); % 's'表示接收字符串
    
    if strcmpi(userInput, 'end')
        % 清空串口对象
        clear serialObj; % 从工作区中删除对象[citation:1][citation:9]
        disp('串口对象已从工作区清空。');
    else
        % 在这里处理其他指令...
        disp(['执行指令: ', userInput]);
    end
end
disp('程序结束。');
%}