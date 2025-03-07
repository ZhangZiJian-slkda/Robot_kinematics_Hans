clc; clear; 

% % 定义标准D-H参数（注意负号需与DH标准定义一致）
% L1 = Link([0, 0.1065, 0, pi/2], 'standard');
% L2 = Link([0, 0, -0.408, 0], 'standard');
% L3 = Link([0, 0, -0.382, 0], 'standard');
% L4 = Link([0, 0.1109, 0, pi/2], 'standard');
% L5 = Link([0, 0.1109, 0, -pi/2], 'standard');
% L6 = Link([0, 0.08409, 0, 0], 'standard');

% 参数顺序：theta, d, a, alpha（标准D-H法）
L1 = Link([0       0.1065  0      pi/2], 'standard');
L2 = Link([0       0      -0.408  0    ], 'standard');
L3 = Link([0       0      -0.382  0    ], 'standard');
L4 = Link([0       0.1109  0      pi/2 ], 'standard');
L5 = Link([0       0.1109  0     -pi/2 ], 'standard');
L6 = Link([0       0.08409 0      0    ], 'standard');

robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6R Robot');

% ------------------- 步骤1：生成目标位姿矩阵 -------------------
q_input = [pi/2, -2*pi/3, -2*pi/3, 0, 2*pi/3, 0];
T_target = robot.fkine(q_input);
disp('目标位姿矩阵：'); disp(T_target);

% ------------------- 步骤2：逆运动学求解 -------------------
tol = 1e-5;     % 误差容忍度
max_iter = 5000; % 最大迭代次数

% 定义关节限制
joint_limits = [-pi pi; -pi pi; -pi pi; -pi pi; -pi pi; -pi pi];
robot.qlim = joint_limits; 

% 关键修正：使用完整的参数名称
opt = optimoptions('fmincon', ...        % 明确指定优化器
    'TolFun', tol, ...                  % 函数值容忍度
    'MaxIter', max_iter, ...            % 最大迭代次数
    'Algorithm', 'sqp');                % 指定算法

% 使用原始输入作为初始猜测
q_guess = q_input; 

% 调用ikcon并传递正确参数
q_ik = robot.ikcon(T_target, q_guess, 'opt', opt);

% ------------------- 步骤3：验证逆解正确性 -------------------
if isempty(q_ik) || numel(q_ik) ~= 6
    error('逆解求解失败，请检查初始值或模型参数！');
end

T_verify = robot.fkine(q_ik);
position_error = norm(T_target.t - T_verify.t);
orientation_error = norm(T_target.R - T_verify.R);

if position_error < tol && orientation_error < tol
    disp('逆解验证成功！');
    disp('求解关节角度（弧度）：'); disp(q_ik);
else
    error('逆解验证失败，位置误差=%.4e，姿态误差=%.4e',...
          position_error, orientation_error);
end

% ------------------- 步骤4：可视化验证 -------------------
figure('Name','原始姿态'); 
robot.plot(q_input, 'workspace', [-1 1 -1 1 -1 1], 'nobase');
title('原始关节角度姿态');
trplot(T_target, 'color', 'r', 'frame', 'T_{target}');

figure('Name','逆解姿态'); 
robot.plot(q_ik, 'workspace', [-1 1 -1 1 -1 1], 'nobase');
title('逆解关节角度姿态');
trplot(T_verify, 'color', 'g', 'frame', 'T_{verify}');