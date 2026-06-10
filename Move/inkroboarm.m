clc; clearvars; close all
 l1 = 0.2; l2 = 0.15; % 后臂和前臂的长度
% serialportlist("available")
 serialObj = serialport("COM22",115200); % 连接后自动reset下位机
%%
while 1 == 1
     str = input('输入坐标:', 's');
     coor = sscanf(str,'%f,%f');
     if isempty(coor)
        break;
     end
     x = coor(1); y = coor(2);
     % …SCARA机械臂右手系逆运动学解算，并发送
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
end
%%

 clear serialObj % 清除了串口,arduino也不运动了
     