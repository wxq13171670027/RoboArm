%{
 这个代码可以实现 对预定义路程的拼接   
%}


%%
clc; clearvars; close all
 l1 = 0.2; l2 = 0.15; % 后臂和前臂的长度
% serialportlist("available")
 serialObj = serialport("COM23",115200); % 连接后自动reset下位机
 %%
 %预定义路程存储
 %{

A
-0.08,0.07;-0.02,0.15;0.12,0.32;0.26,0.15;0.32,0.07;777,777;0.26,0.15;888,888;-0.02,0.15
[888,888;0.07,-0.08;0.15,-0.02;0.32,0.12;0.15,0.26;0.07,0.32;777,777;0.15,0.26;888,888;0.15,-0.02;999,999];

letterPaths.F
letterPaths.U
letterPaths.N
 %}


letterPaths.A = [888,888;0.07,-0.08;0.15,-0.02;0.32,0.12;0.15,0.26;0.07,0.32;777,777;0.15,0.26;888,888;0.15,-0.02;999,999];

letterPaths.B = [888,888;0.33,0.05;0.27,0.05;0.19,0.05;0.13,0.05;0.08,0.05;0.04,0.05;0.09,0.013;0.11,-0.03;0.16,-0.003;0.20,-0.014;0.24,-0.05;0.31,-0.01;0.33,0.05;999,999];

letterPaths.C = [888,888;0.345,-0.187;0.327,-0.115;0.31,-0.07;0.273,-0.023;0.206,0.02;0.13,-0.007;0.106,-0.034;0.069,-0.079;0.03,-0.16;999,999];

formatSpec = '%.2f';

 %%
%将输入的字符串提取出来

inputStr=input('请输入字符串','s');
allPoints = [];
for i = 1:length(inputStr)
    ch = upper(inputStr(i));%都要求是大写
    if isfield(letterPaths, ch)
        pts = letterPaths.(ch);
        allPoints = [allPoints; pts; NaN*ones(1,2)]; % 用 NaN 分隔字母
    end
end

%%
%全局缩放 + 居中 + 从左到右排布，从而生成pur_pos数组
%
cartesianPts = layoutAndScale(allPoints, 0.2, 0.1, 0.1);
%%
%插入控制码
pur_pos = [888,888];
for k = 1:size(allPoints,1)
    if isnan(allPoints(k,1))
        pur_pos = [pur_pos; 777,777];
    else
        pur_pos = [pur_pos; allPoints(k,:)]; %#ok<AGROW>
    end
end
pur_pos = [pur_pos; 999,999];


%%


n=1;
while n <= size(pur_pos,1)
     x = pur_pos(n, 1);
     y = pur_pos(n, 2);
     if x==888 || y==888
         q1 = 888.1;
         q2 = 888.1;
         
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec)]);
         disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
     end
     if x==777 || y==777
         q1 = 777.1;
         q2 = 777.1;
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec)]);
         disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)]);
     end
     if x==999 || y==999
         q1 = 999.1;
         q2 = 999.1;
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec)]);
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
         writeline(serialObj, ['\n',num2str(q1,formatSpec),'\n',num2str(q2,formatSpec)]);
         disp([num2str(q1,formatSpec),',',num2str(q2,formatSpec)])
     end
     n=n+1;
     
end
%%

clear serialObj;

%%
function cartesianPoints = layoutAndScale(allPoints, totalWidth, charHeight, workspaceYCenter)
% 将归一化的字母路径点布局为从左到右、居中的笛卡尔坐标
%
% 输入：
%   allPoints        : N×2 矩阵，含 NaN 分隔符（每个字母后一个 NaN 行）
%   totalWidth       : 所有字母占据的总宽度（米），例如 0.35
%   charHeight       : 单个字母的高度（米），例如 0.15
%   workspaceYCenter : 工作空间Y方向中心（米），例如 0.18
%
% 输出：
%   cartesianPoints  : M×2 矩阵，实际笛卡尔坐标（仍保留 NaN 作为分隔）

if nargin < 2, totalWidth = 0.35; end
if nargin < 3, charHeight = 0.15; end
if nargin < 4, workspaceYCenter = 0.18; end

% Step 1: 按 NaN 分割成字母段
segments = {};
currentSeg = [];
for i = 1:size(allPoints,1)
    if any(isnan(allPoints(i,:)))
        if ~isempty(currentSeg)
            segments{end+1} = currentSeg;
            currentSeg = [];
        end
    else
        currentSeg(end+1,:) = allPoints(i,:); 
    end
end
if ~isempty(currentSeg)
    segments{end+1} = currentSeg;
end

numChars = length(segments);
if numChars == 0
    cartesianPoints = [];
    return;
end

charWidth = totalWidth / numChars; % 每个字母分配的宽度

% Step 2: 对每个字母段进行缩放和平移
scaledSegments = {};
for idx = 1:numChars
    pts = segments{idx}; % 归一化点 [0,1] x [0,1]
    
    % X: 缩放到 [0, charWidth]，然后平移到第 idx 个位置
    x_scaled = pts(:,1) * charWidth + (idx - 1) * charWidth;
    
    % Y: 缩放到 [0, charHeight]
    y_scaled = pts(:,2) * charHeight;
    
    % 合并
    seg_cart = [x_scaled, y_scaled];
    scaledSegments{idx} = seg_cart;
end


% Step 3: 拼接所有段，并计算整体 Y 范围以做垂直居中
fullCart = [];
for idx = 1:numChars
    fullCart = [fullCart; scaledSegments{idx}; NaN*ones(1,2)]; 
end
fullCart(end,:) = []; % 移除最后一个多余的 NaN





% 计算 X 的 min/max
validX = fullCart(~isnan(fullCart(:,2)), 2);
if isempty(validX)
    xOffset = workspaceXCenter;
else
    xMin = min(validX);
    xMax = max(validX);
    xCenter = (xMin + xMax) / 2;
    xOffset = workspaceXCenter - xCenter;
end

% 应用 X偏移（居中）
cartesianPoints = fullCart;
cartesianPoints(~isnan(cartesianPoints(:,2)), 2) = ...
    cartesianPoints(~isnan(cartesianPoints(:,2)), 2) + xOffset;

end