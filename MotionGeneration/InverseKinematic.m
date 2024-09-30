animateOn = true;     % rander the image and show in figure
speedupfactor = 30;   % animation speed up 

% load default parameters if previous script wasn't run
if ~exist('footinfos','var')
    load('defaultfootinfos.mat')
end

% NOTE: make sure parameters match in inverse kinematics function 
L1 = 0.12; 
L2 = 0; 
L3 = 0.4;
L4 = 0.38;
L5 = 0;

robot = rigidBodyTree;

dhparams = [L1      0        -L2     0;     % Base -> hip yaw
            0       -pi/2    0       0;     % Hip yaw -> hip roll
            0       -pi/2    0       0;     % Hip roll -> hip pitch       
            L3      0        0       0;     % Hip pitch -> knee pitch
            L4      0        0       0;     % Knee pitch -> ankle pitch
            0       pi/2     0       0;     % Ankle pitch -> ankle roll
            0       0        L5      0];    % Ankle roll -> end effector (foot)
       
for idx = 1:size(dhparams,1)
    % create a rigidBody, 'rightleg1', 'rightleg2' ....
    rightLeg(idx) = rigidBody("rightleg"+idx); 

    % create a revolute joint
    rightJnt(idx) = rigidBodyJoint("rightjnt"+idx, 'revolute'); 

    % 设置关节的固定变换（基于D-H参数），这定义了关节相对于前一个刚体的位置和方向。
    setFixedTransform(rightJnt(idx),dhparams(idx,:),'dh'); 

    % 将创建的关节对象赋值给对应的刚体。
    rightLeg(idx).Joint = rightJnt(idx);

    % 将刚体添加到机器人模型中。如果是第一个刚体，则将其添加到基座"base"。
    % 否则，将其添加到链中的前一个刚体。
    if idx==1
        addBody(robot,rightLeg(idx),"base");     
    else 
        addBody(robot,rightLeg(idx),"rightleg"+(idx-1));
    end
    
end

dhparams = [-L1     0        -L2     0;    % Only difference with right leg is the
            0       -pi/2    0       0;    % first element is -L1 instead of L1
            0       -pi/2    0       0; 
            L3      0        0       0;
            L4      0        0       0;
            0       pi/2     0       0;
            0       0        L5      0];
        
for idx = 1:size(dhparams,1)
    leftLeg(idx) = rigidBody("leftleg"+idx); 
    leftJnt(idx) = rigidBodyJoint("leftjnt"+idx, 'revolute'); 
    setFixedTransform(leftJnt(idx),dhparams(idx,:),'dh'); 
    leftLeg(idx).Joint = leftJnt(idx);
    if idx==1
        addBody(robot,leftLeg(idx),"base");     
    else
        addBody(robot,leftLeg(idx),"leftleg"+(idx-1));
    end
end

showdetails(robot)    % 显示rigidBodyTree


qright0 = zeros(1,6); 
qleft0 = zeros(1,6); 
% ShowRobotJoints(robot, qright0, qleft0)


% n = [0;  0; -1]; % x
% s = [-1; 0; 0];  % y
% a = [0;  1; 0];  % z
% R = [n s a];   
% for sIdx = 1:length(footinfos)
%     % Extract X Y Z position states (indices 1, 3, and 5)
%     stateL = footinfos{sIdx}.footleft([1 3 5],:); 
%     stateR = footinfos{sIdx}.footright([1 3 5],:); 
% 
%     % Initialize matrices
%     numIdx = size(stateL,2); 
%     jointsLeft = zeros(6,numIdx); 
%     jointsRight = zeros(6,numIdx); 
%     transMatLeft = zeros(4,4,numIdx); 
%     transMatRight = zeros(4,4,numIdx); 
% 
%     % Skip some intermediate steps when visualizing 
%     for idx = 1:numIdx
%         % Get Left joints
%         p = stateL(:,idx); 
%         transmat =  [R     p; 
%                     [0 0 0 1]];
%         isLeft = true; 
%         qLeft = invKinBody2Foot(transmat, isLeft); % Call IK function
%         jointsLeft(:,idx) = qLeft; 
%         transMatLeft(:,:,idx) = transmat; 
% 
%         % Get Right joints
%         p = stateR(:,idx); 
%         transmat =  [R     p; 
%                     [0 0 0 1]];
%         isLeft = false; 
%         qRight = invKinBody2Foot(transmat, isLeft);
%         jointsRight(:,idx) = qRight; 
%         transMatRight(:,:,idx) = transmat; 
% 
%         % Animate
%         if animateOn
%             if rem(idx,speedupfactor) == 0
% 
%                 ShowRobotJoints(robot, qRight, qLeft);
%             end 
%         end
%     end
%     % save joints info 
%     footinfos{sIdx}.jointsleft = jointsLeft; 
%     footinfos{sIdx}.jointsright = jointsRight; 
%     footinfos{sIdx}.transmatleft = transMatLeft; 
%     footinfos{sIdx}.transmatright = transMatRight; 
% end

% last update of Animation in case Animation is off 
% ShowRobotJoints(robot, qRight, qLeft);

qright0 = zeros(1,6); 
qleft0 = zeros(1,6); 
config = GetConfiguration(robot, qright0, qleft0);
pose_link1 = getTransform(robot, config, 'rightleg7', 'base');
disp(pose_link1);
pose_link1 = getTransform(robot, config, 'leftleg7', 'base');

% 显示变换矩阵
disp(pose_link1);

error('test')





% timeVec = footinfos{1}.timevec; 
% transMatLeft = footinfos{1}.transmatleft; 
% transMatRight = footinfos{1}.transmatright; 
% 
% siminL.time = timeVec;
% siminL.signals.values = transMatLeft;
% siminL.signals.dimensions = [4,4];
% 
% siminR.time = timeVec;
% siminR.signals.values = transMatRight;
% siminR.signals.dimensions = [4,4];
% 
% for idx = 2:length(footinfos)
%     timeVec = footinfos{idx}.timevec;
% 
%     transMatLeft = footinfos{idx}.transmatleft; 
%     transMatRight = footinfos{idx}.transmatright; 
% 
%     siminL.time = [siminL.time timeVec];
%     siminL.signals.values = cat(3,siminL.signals.values, transMatLeft);
% 
%     siminR.time = [siminR.time timeVec];
%     siminR.signals.values = cat(3,siminR.signals.values, transMatRight);
% end







function output = GetConfiguration(robot, AngleRight, AngleLeft)
    % get joint home position
    desconfig = robot.homeConfiguration;

    desconfig(1).JointPosition = pi; % angle offset 
    for idx = 1:length(AngleRight)
        desconfig(idx+1).JointPosition = AngleRight(idx);
    end 
    desconfig(2).JointPosition = desconfig(2).JointPosition - pi;
    desconfig(3).JointPosition = desconfig(3).JointPosition + pi/2; 

    desconfig(8).JointPosition = pi;
    for idx = 1:length(AngleLeft)
        desconfig(idx+8).JointPosition = AngleLeft(idx);
    end 
    desconfig(9).JointPosition = desconfig(9).JointPosition - pi; 
    desconfig(10).JointPosition = desconfig(10).JointPosition + pi/2; 
    output = desconfig;
end





function ShowRobotJoints(robot, anglesright, anglesleft)
    desconfig = GetConfiguration(robot, anglesright, anglesleft);

    % update graphics 
    % PreservePlot用来指定是否保留上一帧的绘图, false不保存
    show(robot, desconfig, 'PreservePlot', false);
    title('Walking Pattern Inverse Kinematics')
    pause(0.001)
end





















