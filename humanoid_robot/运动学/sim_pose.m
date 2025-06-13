
% 利用坐标变换关系式，给定全身关节角，模拟 Bioloid 机器人站立动作，区分左脚/右脚支撑

% 运动学方法：DH（Denavit-Hartenberg）或 PB（投影法）
md = 'DH';  
% 支撑模式：左脚支撑（可切换为 'RIGHT_BASE' 表示右脚支撑）
sf = 'LEFT_BASE';  
% 支撑脚中心点位置（参考坐标系原点）
sp = [0, 0, 0];     
% 模拟类型：'pose_sim' 表示静止姿势模拟（可扩展 'walk_sim' 行走、'tread_sim' 原地踏步）
sim_type = 'pose_sim';  

% 导入 Bioloid 机器人规格（连杆长度、质量等），根据 md 选择运动学方法适配的数据
spec = robot_spec_bioloid_prem(md); 
% 提取连杆参数（前 20 个元素）
link = spec(1:20);   
% 提取质量参数（21 - 42 个元素）
mass = spec(21:42);  
% 初始化绘图标志（长度为 5 的 0 数组，控制动作显示细节）
flags = zeros(1,5);  

% 站立姿势 - 矢状面关节角（需结合机器人结构理解各角度对应关节）
th = [pi/2 0 0 pi 0 0 pi pi];  

% 根据支撑模式设置冠状面关节角
if strcmp(sf, 'LEFT_BASE')
    % 左脚支撑时的冠状面关节角
    phi = [0 0 pi/2 0 pi/2 0 pi 0];  
elseif strcmp(sf, 'RIGHT_BASE')
    % 右脚支撑时的冠状面关节角
    phi = [0 0 -pi/2 0 pi/2 0 pi 0];  
end

% 横切面关节角
psi = [0 0];         
% 绝对坐标系与参考坐标系夹角
gamma = 0;           

% 静止姿势时，所有关节角的一阶、二阶微分值设为 0（因为是静态模拟）
th_1d = zeros(1,8);  
th_2d = zeros(1,8);  
phi_1d = zeros(1,8); 
phi_2d = zeros(1,8); 
psi_1d = zeros(1,2); 
psi_2d = zeros(1,2); 
% 模拟图编号（用于区分不同模拟场景的绘图窗口）
fig_no = 1;  

% 根据运动学方法（md）调用对应的运动学计算函数
if strcmp(md, 'DH')==1
    % DH 方法：调用 func_robot_motion_dh 计算运动学
    res = func_robot_motion_dh(link, mass, sf, sp, gamma, th, th_1d, th_2d, ...
        phi, phi_1d, phi_2d, psi, psi_1d, psi_2d, fig_no, sim_type, flags);
% 投影法（PB）分支：若 md 为 'PB'，调用 func_robot_motion_pb 计算
elseif strcmp(md, 'PB')==1
    res = func_robot_motion_pb(link, mass, sf, sp, th, th_1d, th_2d, phi, phi_1d, phi_2d, ...
        psi, fig_no, sim_type, flags);
end
