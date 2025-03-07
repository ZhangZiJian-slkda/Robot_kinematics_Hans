clc
clear;

% 参数顺序：theta, d, a, alpha（标准D-H法）
L1 = Link([0       0.1065  0      pi/2], 'standard');
L2 = Link([0       0      -0.408  0    ], 'standard');
L3 = Link([0       0      -0.382  0    ], 'standard');
L4 = Link([0       0.1109  0      pi/2 ], 'standard');
L5 = Link([0       0.1109  0     -pi/2 ], 'standard');
L6 = Link([0       0.08409 0      0    ], 'standard');

robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'Hans Robot');
disp('机械臂D-H参数表：');
robot.display()

% 计算正运动学
q = [pi/2, -2*pi/3, -2*pi/3, 0, 2*pi/3, 0];
T = robot.fkine(q);
disp('末端执行器位姿矩阵：');
disp(T);

% 可视化机械臂
robot.teach(q, 'workspace', [-1 1 -1 1 -1 1]);
