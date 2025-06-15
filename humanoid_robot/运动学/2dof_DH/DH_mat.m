% DH 变换矩阵计算函数
function y = DH_mat(theta, d, a, alpha)
    % 将角度转换为弧度（如果需要的话）
%     theta = deg2rad(theta);  % 如果传入的是度，转换为弧度
%     alpha = deg2rad(alpha);  % 如果传入的是度，转换为弧度

    % 计算 DH 变换矩阵
    y = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta); 
         0, sin(alpha), cos(alpha), d; 
         0, 0, 0, 1];
end
