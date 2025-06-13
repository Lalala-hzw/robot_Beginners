function y = func_robot_motion_pb(link, mass, sf, sp, th, th_1d, th_2d, phi, phi_1d, phi_2d, psi, fig_no, sim_type, flags)
    g = 9.8062; 
    flag_show_zmp = flags(3);  % 0: 不显示 ZMP，1: 显示 ZMP
    flag_show_2d = flags(4);   % 0: 不显示 2D 平面模型，1: 显示 2D 平面模型
    
    % 矢状面角度（原代码此处仅为注释，可按需补充后续计算逻辑）

% 2.3 利用投影法为仿人机器人建模

% 矢状面角度（推测是基于关节角度 th 计算的累计角度 ）
th1 = th(1); th2 = sum(th(1:2)); th123 = sum(th(1:3)); th1234 = sum(th(1:4)); 
th12345 = sum(th(1:5)); th123456 = sum(th(1:6)); th7 = th(7); th8 = th(8); 
th1237 = th123 + th7; th1238 = th123 + th8;

% 冠状面角度（基于关节角度 phi 计算的累计角度 ）
phi1 = phi(1); phi2 = phi(2); phi3 = phi(3); phi4 = phi(4); phi5 = phi(5); phi6 = phi(6); 
phi7 = phi(7); phi8 = phi(8); phi12 = sum(phi(1:2)); phi123 = sum(phi(1:3)); 
phi1234 = sum(phi(1:4)); phi125 = phi12 + phi5; phi1256 = phi125 + phi6; 
phi127 = phi12 + phi7; phi1278 = phi127 + phi8;

% 连杆长度（从 link 数组中提取各连杆长度参数 ）
l0 = link(1); l1 = link(2); l2 = link(3); l3 = link(4); l4 = link(5); l5 = link(6); 
l6 = link(7); l7 = link(8); l8 = link(9); l9 = link(10); l10 = link(11); l11 = link(12); 
l12 = link(13); l_hd = link(14); l_sh = link(15); l_tr = link(16); 
l_ft_for = link(17); l_ft_back = link(18); l_ft_inner = link(19); l_ft_outer = link(20);

% 连杆质量（从 mass 数组中提取各连杆质量参数 ）
m1 = mass(1); m2 = mass(2); m3 = mass(3); m4 = mass(4); m5 = mass(5); m6 = mass(6); 
m7 = mass(7); m8 = mass(8); m9 = mass(9); m10 = mass(10); m11 = mass(11); m12 = mass(12); 
m13 = mass(13); m14 = mass(14); m15 = mass(15); m16 = mass(16); m18 = mass(18); 
m19 = mass(19); m20 = mass(20); m_hd = mass(21); m_tr = mass(22);

% 下体连杆投影（根据 sf 标识区分左右，计算投影长度 ）
l1s = l1*abs(cos(phi1)); l2s = l2*abs(cos(phi1));
if strcmp(sf, 'LEFT_BASE')
    l4s = l4*abs(cos(phi12+pi/2)); l6s = l6*abs(cos(phi123+pi/2)); 
    l7s = l7*abs(cos(phi123+pi/2)); l8s = l8*abs(cos(phi1234+pi/2));
elseif strcmp(sf, 'RIGHT_BASE')
    l4s = l4*abs(cos(phi12-pi/2)); l6s = l6*abs(cos(phi123-pi/2)); 
    l7s = l7*abs(cos(phi123-pi/2)); l8s = l8*abs(cos(phi1234-pi/2));
end
l1c = l1*abs(sin(th1)); l2c = l2*abs(sin(th2)); l4c = l4; 
l6c = l6*abs(sin(th1234)); l7c = l7*abs(sin(th12345)); l8c = l8*abs(sin(th123456));

% 上体连杆投影（计算上体各部分连杆投影长度 ）
l9s = l9*abs(cos(phi125+pi/2)); l10s = l10*abs(cos(phi1256+pi/2)); 
l11s = l11*abs(cos(phi127)); l12s = l12*abs(cos(phi1278));

l9c = l9*abs(sin(th1237)); l10c = l10*abs(sin(th1237)); 
l11c = l11*abs(sin(th1238)); l12c = l12*abs(sin(th1238));

l_trs = l_tr*cos(phi12); l_trc = l_tr*sin(th123);  % 躯干连杆
l_hds = l_hd*cos(phi12); l_hdc = l_hd*sin(th123);  % 头部连杆
l_shs = l_sh; l_shc = l_sh;  % 肩部连杆（投影计算较简单，可能是简化处理 ）

  % 50 | 第2章 仿人机器人运动学

%% 下体关节坐标
p0 = [sp(1) sp(2) sp(3)+10];
p1 = p0 + [l1s*cos(th1)  -l1c*sin(phi1)  l1s*sin(th1)];
p2 = p1 + [l2s*cos(th2)  -l2c*sin(phi1)  l2s*sin(th2)];

if strcmp(sf, 'LEFT_BASE')
    p3 = p2 + [l4s*cos(th123)   -l4c*sin(phi12+pi/2)  l4s*sin(th123)];
    p4 = p3 + [l6s*cos(th1234)  -l6c*sin(phi123+pi/2) l6s*sin(th1234)];
    p5 = p4 + [l7s*cos(th12345) -l7c*sin(phi123+pi/2) l7s*sin(th12345)];
    p6 = p5 + [l8s*cos(th123456)-l8c*sin(phi1234+pi/2) l8s*sin(th123456)];
elseif strcmp(sf, 'RIGHT_BASE')
    p3 = p2 + [l4s*cos(th123)   -l4c*sin(phi12-pi/2)  l4s*sin(th123)];
    p4 = p3 + [l6s*cos(th1234)  -l6c*sin(phi123-pi/2) l6s*sin(th1234)];
    p5 = p4 + [l7s*cos(th12345) -l7c*sin(phi123-pi/2) l7s*sin(th12345)];
    p6 = p5 + [l8s*cos(th123456)-l8c*sin(phi1234-pi/2) l8s*sin(th123456)];
end

%% 上体关节坐标
pc = (p2+p3)/2;  % 中心坐标
pn = pc + [l_trs*cos(th123)  -l_trc*sin(phi12)  l_trs*sin(th123)];  % 颈部坐标
pt = pn + [l_hds*cos(th123)  -l_hdc*sin(phi12)  l_hds*sin(th123)];  % 头顶坐标
p7 = p7 + [0  l_shc/2*sin(phi12+pi/2)  l_shc/2*sin(phi12)];
p8 = p7 + [l9s*cos(th1237)  -l9c*sin(phi125+pi/2)  l9s*sin(th1237)];
p9 = p8 + [l10s*cos(th1237) -l10c*sin(phi1256+pi/2) l10s*sin(th1237)];
p10 = pn + [0  -l_shc/2*sin(phi12+pi/2)  -l_shc/2*sin(phi12)];
p11 = p10 + [l11s*cos(th1238) -l11c*sin(phi127)  l11s*sin(th1238)];
p12 = p11 + [l12s*cos(th1238) -l12c*sin(phi1278)  l12s*sin(th1238)];

%% 脚掌顶点坐标
if strcmp(sf, 'LEFT_BASE')
    % 支撑腿
    pf_supp = [sp; sp; sp; sp] + [l_ft_for l_ft_outer 0;  % 左脚左前
                                 -l_ft_back l_ft_outer 0;  % 左脚左后
                                 -l_ft_back -l_ft_inner 0; % 左脚右后
                                  l_ft_for -l_ft_inner 0]; % 左脚右前
    % 摆动腿（原代码未完整显示，这里保留结构 ）
    pf_sway = [p6; p6; p6; p6] + ...
        [l_ft_for*cos(th123456 + pi/2) l_ft_inner l_ft_for*sin(th123456 + pi/2);  % 右脚右前
         l_ft_back*cos(th123456 - pi/2) l_ft_inner l_ft_back*sin(th123456 + pi/2); % 右脚右后
         l_ft_back*cos(th123456 - pi/2) -l_ft_outer l_ft_back*sin(th123456 - pi/2); % 右脚左后
         l_ft_for*cos(th123456 + pi/2) -l_ft_outer l_ft_back*sin(th123456 - pi/2)]; % 右脚左前（原代码可能有排版截断，需结合完整逻辑校验 ）
end

  %% 2.3 利用投影法为仿人机器人建模 | 51 |
elseif strcmp(sf, 'RIGHT_BASE')
    % 支撑腿（右脚支撑时各顶点坐标计算 ）
    pf_supp = [sp; sp; sp; sp] + [l_ft_for l_ft_inner 0;  % 右脚左前
                                 -l_ft_back l_ft_inner 0;  % 右脚左后
                                 -l_ft_back -l_ft_outer 0; % 右脚右后
                                  l_ft_for -l_ft_outer 0]; % 右脚右前
    
    % 摆动腿（原代码未完整显示，这里保留结构并按逻辑补全 ）
    pf_sway = [p6; p6; p6; p6] + ...
        [l_ft_for*cos(th123456 + pi/2) l_ft_outer l_ft_for*sin(th123456 + pi/2);  % 左脚左前
         l_ft_back*cos(th123456 - pi/2) l_ft_outer l_ft_back*sin(th123456 - pi/2); % 左脚左后
         l_ft_back*cos(th123456 - pi/2) -l_ft_inner l_ft_back*sin(th123456 - pi/2); % 左脚右后
         l_ft_for*cos(th123456 + pi/2) -l_ft_inner l_ft_for*sin(th123456 + pi/2)]; % 左脚右前
end

% 脚掌最低点（取摆动腿脚掌 z 坐标最小值 ）
tip_z_min = min(pf_sway(:,3));  
% 摆动腿的脚部中心点（提取 p6 对应坐标 ）
tip_x = p6(1); tip_y = p6(2); tip_z = p6(3);  
