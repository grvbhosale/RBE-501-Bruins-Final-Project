clear;clc;close all

s=1;  % resolution factor for to make frames visible

dhparams = [0/s         pi/2	0/s	pi/2;
    0.9144/s	0       0/s	0;  % Basic 2d planar robot dh table
    0.9144/s	0       0/s	0;
    0           0       0/s	0];

robot = robotics.RigidBodyTree; % Robot structure within RST

%Assign names to bodies and joints and determine type
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','fixed');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('tool');
jnt4 = robotics.Joint('jnt4','fixed');

addVisual(body2,"Mesh",'link_body.stl')

addCollision(body3,collisionCylinder(0.127,0.9144));
%Calculate transformation matrices for each joint
setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');

%Assign joints to each link body
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;

%Set limits of each joint
jnt2.PositionLimits = [-pi pi];
jnt3.PositionLimits = [-pi pi];


%Create each link's body within the robot structure
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')

%Assigned joint velocities for each joint
q_dot_1 = 1.5; %rad/s
q_dot_2 = 1.5; %rad/s

%Conditions for starting robot pose before sensor reads ball
tsim = 0;
config = homeConfiguration(robot);
config(1).JointPosition = 5*pi/8;
config(2).JointPosition = -5*pi/8;
initialguess = config;
show(robot,config,'Visuals','on','Collisions','on')
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.0025 0.0025 0.0025 0.01 0.01 0.01];

%Create trajectory
baseball = Ball(.6, .06); %mass(kg),radius(meters)
% 
trajCalc = Trajectory_Calculator(baseball, 0, .001);

timeOfFlight = 1;%sec
degreeOfRandomness = 2;%meters
initPos = [-20, 0, 1];%meters
workspaceCenter = [0, 0, 0];%meters
workspaceRadius = 1.5;%meters
[windVel, initVel, actualError, actualTimeOfFlight, initPos] = trajCalc.generateParameters(timeOfFlight, degreeOfRandomness, initPos, workspaceCenter, workspaceRadius);

% Hard coded, non random
initPos = [-18.44, -4, -15];
initVel = [10, 0, 15];
windVel = [5, 6, 0];

traj = trajCalc.calcTrajectory(initPos, initVel, windVel); %full trajectory

%Get current ball position
ball_pos = trajCalc.getBallPos(0);
tsim_ite_str = 0;
tsim_ite_end = 0;
full_path = [];
full_pred = [];
full_move = [];
%Position iteration for the system including sensor and arm
while ball_pos(1) < 0 % Continue until z value is pass the plane of the
% robot for ball trajectory
%     t_range = linspace(tsim_ite_str,tsim_ite_end,50);
%     for p = 1:length(t_range)
%         tr = t_range(p);
%         ball_pos = trajCalc.getBallPos(tr);
%         [y, z] = trajCalc.predictParabolic(tr);
%         ball_pred_pos = [0, y, z];
%         full_path = [full_path,[ball_pos(1);ball_pos(2);ball_pos(3)]];
%         full_pred = [full_pred,ball_pred_pos];
%     end

    tic  %starts simulated time for interval between sensor determining
    %the trajectory of the ball and the arm's brain calculating the
    %next pose

    %Get current ball position and predicted position
    ball_pos = trajCalc.getBallPos(tsim);
    [intersectTime, y, z] = trajCalc.predictParabolic(tsim);
    ball_pred_pos = [0, y, z]
    
    full_path = [full_path,[ball_pos(1);ball_pos(2);ball_pos(3)]];
    full_pred = [full_pred,ball_pred_pos];

    [configSol,solInfo1] = ik('tool',trvec2tform(ball_pred_pos),weights,initialguess);
    robot.DataFormat = 'row';
    if tsim == 0  %Initial time when the sensor first determines the
        %trajectory of the ball, the arm does not move yet
        tsim = toc; %Time of initial reading
        int = toc; %Stores the interval for next iteration to calculate the
                    %first move by the arm
        robot.DataFormat = 'struct';
        scatter3(0, y, z,'o','MarkerEdgeColor','r')
        hold on
        scatter3(ball_pos(1), ball_pos(2), ball_pos(3),'o','MarkerEdgeColor','b')
        show(robot,config);
        hold off
    else
        configstart = config; %determines that last pose made by the are as
        %the starting point for this iteration

        %Changes the current pose of the robot based on time elapsed
        %between each calulation of the configuration solution. It rewrites
        %the configuration variables which are then used for the inverse
        %kinematic calculation of the next interval
        delta_j1 = abs(configSol(1,1).JointPosition -config(1,1).JointPosition);
        delta_j2 = abs(configSol(1,2).JointPosition -config(1,2).JointPosition);
        
        if delta_j1 >= q_dot_1*int
            config(1,1).JointPosition = config(1,1).JointPosition + ...
            ((configSol(1,1).JointPosition - configstart(1,1).JointPosition)/...
            abs(configstart(1,1).JointPosition - configSol(1,1).JointPosition))...
            *q_dot_1*int;
        else
            config(1,1).JointPosition = config(1,1).JointPosition +...
                (configSol(1,1).JointPosition -config(1,1).JointPosition);
        end
        
        if delta_j2 >= q_dot_2*int
        config(1,2).JointPosition = config(1,2).JointPosition + ...
            ((configSol(1,2).JointPosition - configstart(1,2).JointPosition)/...
            abs(configstart(1,2).JointPosition - configSol(1,2).JointPosition))...
            *q_dot_2*int;
        else
            config(1,2).JointPosition = config(1,2).JointPosition +...
                (configSol(1,2).JointPosition -config(1,2).JointPosition);
        end

        %Stores the interval time for next iteration and total time elapsed
        int = toc;
        
        full_move = [full_move,[configstart(1,1).JointPosition,config(1,1).JointPosition;...
            configstart(1,2).JointPosition,config(1,2).JointPosition]];
        
%         q1_range = linspace(configstart(1,1).JointPosition,config(1,1).JointPosition,50);
%         q2_range = linspace(configstart(1,2).JointPosition,config(1,2).JointPosition,50);
%         for q = 1:length(q1_range)
%             full_move = [full_move,[q1_range(q);q2_range(q)]];
%         end
        
%         robot.DataFormat = 'struct';
% %         stg = linspace(tsim1,tsim2);
% %         for j = length(stg)
%             scatter3(0, y, z,'o','MarkerEdgeColor','r');
%             hold on
%             scatter3(ball_pos(1), ball_pos(2), ball_pos(3),'o','MarkerEdgeColor','b')
%             show(robot,config);
%             xlim([-20 20])
%             ylim([-20 20])
%             zlim([-20 20])
%             hold off
%             waitfor(rateControl(5));
            tsim = tsim + int;
    end
end

% full_path = unique(full_path);
% full_pred = unique(full_pred);
% full_move = unique(full_move);
% full_path_com = [];
% full_pred_com = [];
% full_move_com = [];
% for n = 1:length(full_path)-1
%     full_path_com = [full_path_com,linspace(full_path(n),full_path(n+1))];
%     full_pred_com = [full_pred_com,linspace(full_pred(n),full_pred(n+1))];
%     full_move_com = [full_move_com,linspace(full_move(n),full_move(n+1))];
% end

robot.DataFormat = 'struct';
transform = getTransform(robot,config,'tool','base');
y_ee = transform(2,4);
z_ee = transform(3,4);
error_catch_y = abs(traj(end,3) - y_ee);
error_catch_z = abs(traj(end,4) - z_ee);
if error_catch_y > 0.1 || error_catch_z > 0.1
    fprintf('Missed\n')
    scatter3(0, y, z,'o','MarkerEdgeColor','r');
    hold on
    show(robot,config);
    xlim([-10 10])
    ylim([-10 10])
    zlim([-10 10])
    hold off
else
    fprintf('Caught\n')
    scatter3(0, y, z,'o','MarkerEdgeColor','g');
    hold on
    show(robot,config);
    xlim([-10 10])
    ylim([-10 10])
    zlim([-10 10])
    hold off
    waitfor(rateControl(1));
end
