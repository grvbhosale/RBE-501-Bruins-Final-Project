clear;clc;
s=10;  % resolution factor for to make frames visible

dhparams = [0/s     pi/2	0/s	0;
            10/s	0       0/s	0;  % Basic 2d planar robot dh table
            10/s	0       0/s	0;
            0       0       0/s	0];
        
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
jnt2.PositionLimits = [0 pi];
jnt3.PositionLimits = [-pi pi];

%Create each link's body within the robot structure
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')

%Assigned joint velocities for each joint
q_dot_1 = 0.03; %m/s
q_dot_2 = 0.03; %m/s

%Conditions for starting robot pose before sensor reads ball
t = 0;
initialguess = robot.homeConfiguration; 
config = initialguess;
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.0025 0.0025 0.0025 0.01 0.01 0.01];
%Position iteration for the system including sensor and arm
% while ball_pos(2) > 0 % Continue until z value is pass the plane of the
                        % robot for ball trajectory
for i = 1:10  %for loop just for testing interval calculations
    robot.DataFormat = 'row';
    tic  %starts simulated time for interval between sensor determining
         %the trajectory of the ball and the arm's brain calculating the
         %next pose
         
    %Code for ball movement here!!!!
    %
    %
    %
    ball_pos = [0.6 0 0];
    [configSol,solInfo1] = ik('tool',trvec2tform(ball_pos),weights,config);
    robot.DataFormat = 'row';
    if t == 0  %Initial time when the sensor first determines the
               %trajectory of the ball, the arm does not move yet
        t = toc; %Time of initial reading
        int = toc; %Stores the interval for next iteration to calculate the
                   %first move by the arm
    else
        configstart = config; %determines that last pose made by the are as
                              %the starting point for this iteration
        
        %Changes the current pose of the robot based on time elapsed
        %between each calulation of the configuration solution. It rewrites
        %the configuration variables which are then used for the inverse
        %kinematic calculation of the next interval
        config(1,1).JointPosition = config(1,1).JointPosition + ...
        ((configstart(1,1).JointPosition - configSol(1,1).JointPosition)/...
        abs(configstart(1,1).JointPosition - configSol(1,1).JointPosition))...
        *q_dot_1*int;
    
        config(1,2).JointPosition = config(1,2).JointPosition + ...
        ((configstart(1,2).JointPosition - configSol(1,2).JointPosition)/...
        abs(configstart(1,2).JointPosition - configSol(1,2).JointPosition))...
        *q_dot_2*int;
        
        %Stores the interval time for next iteration and total time elapsed
        int = toc;
        t = t + int;
    end
end
robot.DataFormat = 'struct';
show(robot,configSol);



