function y = func_robot_motion_dh(link, mass, sf, sp, gamma, th, th_1d, th_2d, phi, phi_1d, phi_2d, psi, psi_1d, psi_2d, fig_no, sim_type, flags)
    % 重力加速度
    g = 9.8062; 
    % 提取 ZMP 显示标志（用于后续可能的 ZMP 可视化等）
    flag_show_zmp = flags(3); 
    % 提取轨迹显示标志（用于控制轨迹绘制等逻辑）
    flag_show_trace = flags(5); 

    % 矢状面角度：从 th 数组中拆解各关节角度，th 存储矢状面关节角序列
    th1 = th(1); th2 = th(2); th3 = th(3); th4 = th(4); 
    th5 = th(5); th6 = th(6); th7 = th(7); th8 = th(8);

    % 冠状面角度：从 phi 数组中拆解各关节角度，phi 存储冠状面关节角序列
    phi1 = phi(1); phi2 = phi(2); phi3 = phi(3); phi4 = phi(4); 
    phi5 = phi(5); phi6 = phi(6); phi7 = phi(7); phi8 = phi(8);

    % 横切面角度：从 psi 数组中拆解各关节角度，psi 存储横切面关节角序列
    psi1 = psi(1); psi2 = psi(2);

    % 连杆长度：从 link 数组中提取各连杆长度参数，按机器人结构定义顺序
    l0 = link(1); l1 = link(2); l2 = link(3); l3 = link(4); 
    l4 = link(5); l5 = link(6); l6 = link(7); l7 = link(8); 
    l8 = link(9); l9 = link(10); l10 = link(11); l11 = link(12); 
    l12 = link(13); l_hd = link(14); l_sh = link(15); l_tr = link(16); 
    l_ft_for = link(17); l_ft_back = link(18); l_ft_inner = link(19); 
    l_ft_outer = link(20);

    % 连杆质量：从 mass 数组中提取各连杆质量参数，按机器人结构定义顺序
    m1 = mass(1); m2 = mass(2); m3 = mass(3); m4 = mass(4); 
    m5 = mass(5); m6 = mass(6); m7 = mass(7); m8 = mass(8); 
    m9 = mass(9); m10 = mass(10); m11 = mass(11); m12 = mass(12); 
    m13 = mass(13); m14 = mass(14); m15 = mass(15); m16 = mass(16); 
    m18 = mass(18); m19 = mass(19); m20 = mass(20);

    % 连杆坐标系的原点坐标（齐次坐标形式，用于 DH 变换等运动学计算 ）
    O = [0 0 0 1]'; 
    % 连杆坐标系的 z 轴坐标（齐次坐标形式，辅助定义连杆坐标系方向 ）
    Z = [0 0 1 1]'; 
    % 后续需补充连杆重心坐标的计算逻辑，这里仅做注释占位 
    % 连杆坐标系的连杆重心坐标 

    % 2.2 利用 DH 方法为仿人机器人建模
% 坐标变换与关节链计算：支撑腿、摆动腿关节坐标解算

% 局部坐标系参数定义（C1-C20 为连杆坐标系原点，依赖机器人结构 ）
C1 = [0 0 0 1]'; C2 = [-l1/2 0 0 1]'; C3 = [-l2/2 0 0 1]'; C4 = [0 0 0 1]'; 
C5 = [0 0 -l3/2 1]'; C6 = [-l4/2 0 0 1]';
if strcmp(sf, 'LEFT_BASE')
    C7 = [0 -l5/2 0 1]';
elseif strcmp(sf, 'RIGHT_BASE')
    C7 = [0 l5/2 0 1]';
end
C8 = [0 0 0 1]'; C9 = [-l6/2 0 0 1]'; C10 = [-l7/2 0 0 1]'; C11 = [0 0 0 1]'; 
C12 = [-l8/2 0 0 1]'; C13 = [-l_tr/2 0 -l_sh/2 1]'; C14 = [-0.014/2 -0.025/2 0 1]'; 
C15 = [-l9/2 0 0 1]'; C16 = [-l10/2 0 0 1]'; C18 = [-0.014/2 0.025/2 0 1]'; 
C19 = [-l11/2 0 0 1]'; C20 = [-l12/2 0 0 1]';

% 全局坐标系变换（补偿 gamma 角，sp 为支撑脚中心点 ）
W = [-sin(gamma) 0 cos(gamma) 0; cos(gamma) 0 sin(gamma) 0; 0 1 0 10; 0 0 0 1];
G = [0 -g 0]';  % 重力向量（简化用）
O0_0 = O; Z0_0 = Z;
O0_ow = W*O + [sp 1]';  % 基坐标系原点在绝对坐标系的位置

% ---------------------- 下体支撑腿关节坐标（DH 变换链） ----------------------
% 关节 1-6：支撑腿主链
B1 = DH_mat(phi1+pi/2, 0, 0, pi/2); T1_0 = B1;
O1_0 = T1_0*O; O1_ow = W*O1_0 + [sp 1]';

B2 = DH_mat(-th1+pi/2, 0, l1, 0); T2_0 = T1_0*B2;
O2_0 = T2_0*O; O2_ow = W*O2_0 + [sp 1]';

B3 = DH_mat(-th2, 0, l2, 0); T3_0 = T2_0*B3;
O3_0 = T3_0*O; O3_ow = W*O3_0 + [sp 1]';

B4 = DH_mat(-th3, 0, 0, -pi/2); T4_0 = T3_0*B4;
O4_0 = T4_0*O; O4_ow = W*O4_0 + [sp 1]';

B5 = DH_mat(phi2, 0, l3, 0); T5_0 = T4_0*B5;
O5_0 = T5_0*O; O5_ow = W*O5_0 + [sp 1]';

% 支撑腿分支（左/右脚判断）
if strcmp(sf, 'LEFT_BASE')
    B6 = DH_mat(psi1, 0, l4, 0);
elseif strcmp(sf, 'RIGHT_BASE')
    B6 = DH_mat(psi1+pi, 0, l4, 0);
end
T6_0 = T5_0*B6; O6_0 = T6_0*O; O6_ow = W*O6_0 + [sp 1]';

% ---------------------- 下体摆动腿关节坐标（DH 变换链） ----------------------
if strcmp(sf, 'LEFT_BASE')
    B7 = DH_mat(psi2, 0, 0, -pi/2);
    T7_0 = T6_0*B7*[1 0 0 0; 0 1 0 15; 0 0 1 0; 0 0 0 1];  % 补偿摆动腿偏移
elseif strcmp(sf, 'RIGHT_BASE')
    B7 = DH_mat(psi2, 0, 0, pi/2);
    T7_0 = T6_0*B7*[1 0 0 0; 0 1 0 -15; 0 0 1 0; 0 0 0 1]; % 补偿摆动腿偏移
end
O7_0 = T7_0*O; O7_ow = W*O7_0 + [sp 1]';

B8 = DH_mat(phi3, 0, 0, -pi/2); T8_0 = T7_0*B8;
O8_0 = T8_0*O; O8_ow = W*O8_0 + [sp 1]';


% 36 | 第2章 仿人机器人运动学
% 摆动腿脚掌 + 上体左胳膊关节坐标计算

% ---------------------- 摆动腿关节链延续（DH 变换） ----------------------
B9 = DH_mat(-th4+pi, 0, 16, 0); T9_0 = T8_0*B9; 
O9_0 = T9_0*O; O9_ow = W*O9_0 + [sp 1]';

B10 = DH_mat(-th5, 0, 17, 0); T10_0 = T9_0*B10; 
O10_0 = T10_0*O; O10_ow = W*O10_0 + [sp 1]';

B11 = DH_mat(-th6, 0, 0, pi/2); T11_0 = T10_0*B11; 
O11_0 = T11_0*O; O11_ow = W*O11_0 + [sp 1]';

B12 = DH_mat(phi4, 0, 18, 0); T12_0 = T11_0*B12; 
O12_0 = T12_0*O; O12_ow = W*O12_0 + [sp 1]';

% 摆动脚掌中心点
tip_x = O12_ow(1); tip_y = O12_ow(2); tip_z = O12_ow(3);

% ---------------------- 脚掌顶点坐标（支撑腿 + 摆动腿） ----------------------
if strcmp(sf, 'LEFT_BASE')
    % 支撑腿（左脚）4 个顶点
    pf_0_1w = W*[l_ft_outer; -10; l_ft_for; 1] + [sp; 1];
    pf_0_2w = W*[-l_ft_inner; -10; l_ft_for; 1] + [sp; 1];
    pf_0_3w = W*[-l_ft_inner; -10; -l_ft_back; 1] + [sp; 1];
    pf_0_4w = W*[l_ft_outer; -10; -l_ft_back; 1] + [sp; 1];

    % 摆动腿（右脚）4 个顶点
    pf_12_1 = T12_0*[0; l_ft_inner; l_ft_for; 1];
    pf_12_2 = T12_0*[0; -l_ft_outer; l_ft_for; 1];
    pf_12_3 = T12_0*[0; -l_ft_outer; -l_ft_back; 1];
    pf_12_4 = T12_0*[0; l_ft_inner; -l_ft_back; 1];
elseif strcmp(sf, 'RIGHT_BASE')
    % 支撑腿（右脚）4 个顶点
    pf_0_1w = W*[l_ft_inner; -10; l_ft_for; 1] + [sp; 1];
    pf_0_2w = W*[-l_ft_outer; -10; l_ft_for; 1] + [sp; 1];
    pf_0_3w = W*[-l_ft_outer; -10; -l_ft_back; 1] + [sp; 1];
    pf_0_4w = W*[l_ft_inner; -10; -l_ft_back; 1] + [sp; 1];

    % 摆动腿（左脚）4 个顶点
    pf_12_1 = T12_0*[0; l_ft_outer; l_ft_for; 1];
    pf_12_2 = T12_0*[0; -l_ft_inner; l_ft_for; 1];
    pf_12_3 = T12_0*[0; -l_ft_inner; -l_ft_back; 1];
    pf_12_4 = T12_0*[0; l_ft_outer; -l_ft_back; 1];
end

% ---------------------- 上体左胳膊关节坐标（DH 变换） ----------------------
if strcmp(sf, 'LEFT_BASE')
    T13_6 = [0, 0, -1, -14/2 - l_sh/2; ...
             0, 1, 0, 0; ...
             1, 0, 0, l_tr; ...
             0, 0, 0, 1];
else
    T13_6 = [0, 0, 1, -14/2 + l_sh/2; ...
             0, -1, 0, 0; ...
             1, 0, 0, l_tr; ...
             0, 0, 0, 1];
end
T13_0 = T6_0*T13_6;  O13_0 = T13_0*O;  O13_ow = W*O13_0 + [sp; 1];

B14 = DH_mat(-th7, 0.025, 0.014, pi/2);  T14_0 = T13_0*B14;
O14_0 = T14_0*O;  O14_ow = W*O14_0 + [sp; 1];


% 37 | 2.2 利用 DH 方法为仿人机器人建模
% 上体右胳膊关节坐标 + 身体特征点 + 可视化配置

% ---------------------- 上体右胳膊关节坐标（DH 变换） ----------------------
O14_ow = W*O14_0 + [sp 1]';

B15 = DH_mat(phi5-pi/2, 0, 19, 0); T15_0 = T14_0*B15; 
O15_0 = T15_0*O; O15_ow = W*O15_0 + [sp 1]';

B16 = DH_mat(phi6, 0, 110, 0); T16_0 = T15_0*B16; 
O16_0 = T16_0*O; O16_ow = W*O16_0 + [sp 1]';

% 右胳膊分支（依赖支撑腿方向）
if strcmp(sf, 'LEFT_BASE')
    T17_6 = [0, 0, -1, -14/2 + l_sh/2; ...
             0, 1, 0, 0; ...
             1, 0, 0, l_tr; ...
             0, 0, 0, 1];
else
    T17_6 = [0, 0, 1, -14/2 - l_sh/2; ...
             0, -1, 0, 0; ...
             1, 0, 0, l_tr; ...
             0, 0, 0, 1];
end
T17_0 = T6_0*T17_6;  O17_0 = T17_0*O;  O17_ow = W*O17_0 + [sp; 1];

B18 = DH_mat(-th8, -0.025, 0.014, pi/2);  T18_0 = T17_0*B18;
O18_0 = T18_0*O;  O18_ow = W*O18_0 + [sp; 1];

B19 = DH_mat(phi7-pi, 0, 111, 0);  T19_0 = T18_0*B19;
O19_0 = T19_0*O;  O19_ow = W*O19_0 + [sp; 1];

B20 = DH_mat(phi8, 0, 112, 0);  T20_0 = T19_0*B20;
O20_0 = T20_0*O;  O20_ow = W*O20_0 + [sp; 1];

% ---------------------- 身体特征点计算 ----------------------
pc = (O5_ow + O6_ow)/2;   % 身体中心点
pn = (O13_ow + O17_ow)/2; % 肩颈点
pt = W*T6_0*[-14/2; 0; l_tr + l_hd; 1] + [sp; 1]; % 前胸点

% 输出结果（简化，需补充 ZMP 等完整逻辑）
y = [tip_x, tip_y, tip_z, tip_z_min, zmp_x, zmp_y, pc(1)];

% ---------------------- 可视化颜色配置（左/右腿区分） ----------------------
if strcmp(sf, 'LEFT_BASE')
    % 支撑腿（左）：蓝绿色；摆动腿（右）：红
    c_sup_lg1 = '-bo'; c_sup_lg2 = '-b'; 
    c_sway_lg1 = '-ro'; c_sway_lg2 = '-r';
elseif strcmp(sf, 'RIGHT_BASE')
    % 支撑腿（右）：红；摆动腿（左）：蓝绿色
    c_sup_lg1 = '-ro'; c_sup_lg2 = '-r'; 
    c_sway_lg1 = '-bo'; c_sway_lg2 = '-b';
end

% ---------------------- 姿态可视化（核心绘图逻辑） ----------------------
if fig_no >= 1
    figure(fig_no); clf; hold on; grid on; axis equal;
    view(45, 20);  % 3D 视角

    % 辅助函数：处理小坐标值（避免 plot3 中断）
    mz = @(x) x .* (abs(x) > 1e-10) + 0 .* (abs(x) <= 1e-10);

    % 1. 支撑腿关节链绘图
    plot3(mz([sp(1); O0_ow(1); O1_ow(1); O2_ow(1); O3_ow(1); O4_ow(1); O5_ow(1); pc(1)]), ...
          mz([sp(2); O0_ow(2); O1_ow(2); O2_ow(2); O3_ow(2); O4_ow(2); O5_ow(2); pc(2)]), ...
          mz([sp(3); O0_ow(3); O1_ow(3); O2_ow(3); O3_ow(3); O4_ow(3); O5_ow(3); pc(3)]), ...
          c_sup_lg1, 'LineWidth', 1.5);

  %摆动腿关节链（身体侧）
    plot3(mz([pc(1); O6_ow(1); O7_ow(1); O8_ow(1); O9_ow(1); O10_ow(1); O11_ow(1); O12_ow(1)]), ...
          mz([pc(2); O6_ow(2); O7_ow(2); O8_ow(2); O9_ow(2); O10_ow(2); O11_ow(2); O12_ow(2)]), ...
          mz([pc(3); O6_ow(3); O7_ow(3); O8_ow(3); O9_ow(3); O10_ow(3); O11_ow(3); O12_ow(3)]), ...
          c_sway_lg1, 'LineWidth', 1.5);

    % 2. 左胳膊关节链
    plot3(mz([pn(1); O13_ow(1); O14_ow(1); O15_ow(1); O16_ow(1)]), ...
          mz([pn(2); O13_ow(2); O14_ow(2); O15_ow(2); O16_ow(2)]), ...
          mz([pn(3); O13_ow(3); O14_ow(3); O15_ow(3); O16_ow(3)]), ...
          '-bo', 'LineWidth', 1.5);

    % 3. 右胳膊关节链
    plot3(mz([pn(1); O17_ow(1); O18_ow(1); O19_ow(1); O20_ow(1)]), ...
          mz([pn(2); O17_ow(2); O18_ow(2); O19_ow(2); O20_ow(2)]), ...
          mz([pn(3); O17_ow(3); O18_ow(3); O19_ow(3); O20_ow(3)]), ...
          '-ro', 'LineWidth', 1.5);

    % 4. 支撑脚脚掌顶点
    plot3(mz([pf_0_1w(1); pf_0_2w(1); pf_0_3w(1); pf_0_4w(1); pf_0_1w(1)]), ...
          mz([pf_0_1w(2); pf_0_2w(2); pf_0_3w(2); pf_0_4w(2); pf_0_1w(2)]), ...
          mz([pf_0_1w(3); pf_0_2w(3); pf_0_3w(3); pf_0_4w(3); pf_0_1w(3)]), ...
          c_sup_lg2, 'LineWidth', 1.5);

    % 5. 摆动脚脚掌顶点
    plot3(mz([pf_12_1(1); pf_12_2(1); pf_12_3(1); pf_12_4(1); pf_12_1(1)]), ...
          mz([pf_12_1(2); pf_12_2(2); pf_12_3(2); pf_12_4(2); pf_12_1(2)]), ...
          mz([pf_12_1(3); pf_12_2(3); pf_12_3(3); pf_12_4(3); pf_12_1(3)]), ...
          c_sway_lg2, 'LineWidth', 1.5);

    % 6. 身体特征点（中心、肩颈、前胸）
    plot3(mz([pc(1); pn(1); pt(1)]), ...
          mz([pc(2); pn(2); pt(2)]), ...
          mz([pc(3); pn(3); pt(3)]), ...
          '-ko', 'MarkerSize', 8);

    % ---------------------- 轴范围与视角配置 ----------------------
    if strcmp(sim_type, 'walk_sim')
        axis([-0.2 1 -0.2 0.2 0 0.5]);  % 行走模拟轴范围
    elseif strcmp(sim_type, 'tread_sim')
        axis([-0.4 0.4 -0.2 0.2 0 0.5]); % 原地踏步轴范围
    else
        axis([-0.2 0.2 -0.2 0.2 0 0.5]); % 默认轴范围
    end

    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');  % 坐标轴标签
    grid on; hold on;
end
