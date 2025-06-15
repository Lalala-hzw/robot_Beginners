% 设置 DH 参数
d1 = 0; d2 = 0;  % 连杆长度
a1 = 0.1; a2 = 0.2;  % 连杆偏移
alpha1 = 0; alpha2 = 0;  % 连杆扭转角
theta1 = deg2rad(20:1:50);  % 关节角度，从 0 到 150 度，步长为 1
theta2 = deg2rad(0:1:30); 
% 初始化位置变量（使用齐次坐标形式）
p0 = [0; 0; 0; 1];
p1 = [0; 0; 0; 1];  % p1 表示基坐标系中的位置，使用 4x1 向量
p2 = [0; 0; 0; 1];  % p2 表示第二个坐标系中的位置，使用 4x1 向量

% 循环绘制机器人关节的运动

for i = 1:length(theta1)
    % 计算第一个坐标系的位置
    p1_0 = DH_mat(theta1(i), d1, a1, alpha1) * p1;  
    % 计算第二个坐标系的位置
    p2_0 = DH_mat(theta1(i), d1, a1, alpha1)*DH_mat(theta2(i), d2, a2, alpha2) * p2;
    
    % 绘制当前位置
    figure(1);
    plot([p0(1) p1_0(1) p2_0(1)], [p0(2) p1_0(2) p2_0(2)],'-bo');  % 绘制连接 p1_0 和 p2_0 的线段
    xlabel('x (m)');
    ylabel('y (m)');
    hold on;
end

