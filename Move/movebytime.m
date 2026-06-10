clc; clearvars; close all
 l1 = 0.2; l2 = 0.15; % 后臂和前臂的长度
% serialportlist("available")
 serialObj = serialport("COM23",115200); % 连接后自动reset下位机

 %%

P1 = [0.35, 0];
P2 = [0.12, 0.14];


%coeff = polyfit(x, y, 2);

 % 生成20个点（包括端点）
t = linspace(0, 1, 30);
x = P1(1) + t * (P2(1) - P1(1));
y = P1(2) + t * (P2(2) - P1(2));

% 合并为 N×2 的点数组
points1 = [x', y'];

points2 = [x', -1*y'];

P3 = [0.2, 0.09];
P4 = [0.2, -0.09];

% 生成10个插值参数 t ∈ [0, 1]
t3 = linspace(0, 1, 30);

% 线性插值计算 x 和 y 坐标
x3 = P3(1) + t3 * (P4(1) - P3(1));
y3 = P3(2) + t3 * (P4(2) - P3(2));

% 合并为 10×2 的点矩阵
points3 = [x3', y3'];


 %%
 
formatSpec = '%.2f';

%pur_pos = [888,888;0.3,0;0.17,0.17;0.1,0.1;0.03,0.07;0.1,0;999,999];
pur_pos =[888,888;points1;777,777;points2(1,:);888,888;points2(2:end,:);777,777;points3(1,:);888,888;points3(2:end,:);999,999];
%pur_pos =[888,888;points1;999,999];


%pur_pos =[888,888;points3(2:end,:);999,999];

n=1;
while n <= size(pur_pos,1)
     x = pur_pos(n, 1);
     y = pur_pos(n, 2);
     if x==888 || y==888
         q1 = 888.1;
         q2 = 888.1;
         
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec),'\n',num2str(0.1)]);
         disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
     end
     if x==777 || y==777
         q1 = 777.1;
         q2 = 777.1;
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec),'\n',num2str(0.1)]);
         disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
     end
     if x==999 || y==999
         q1 = 999.1;
         q2 = 999.1;
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec),'\n',num2str(0.1)]);
         disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)])
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
         % disp([q1 q2])
         if pur_pos(n-1, 1)==888  || pur_pos(n-2, 1)==888
            writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec),'\n',num2str(0.5)]);
            disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
         elseif pur_pos(n-1, 1)==777 ||  pur_pos(n-2, 1)==777
            writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec),'\n',num2str(1)]);
            disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
         else
            writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec),'\n',num2str(0.1)]);
            disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
         end
         
     end
     n=n+1;
     
end
%%

clear serialObj;
