clc
clear
clf
close

%Test trajectory generation
catcher = BallCatcher();
ticTest = tic;
ticTest2 = tic;
for i = .1:.01:3
    initializeTrajectoryFast(catcher, i);
    if(rem(i,1) ==0)
        timeInt = toc(ticTest2);
        fprintf("%d, Average Time: %d\n", i, timeInt/100)
        ticTest2 = tic;
    end
end
toc(ticTest)
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Core section for gathering data this section will record the initial
% parameters based on 
type = "ik";  % "ik" - inverse kinematics / "anfis" - anfis fussy prediction
n = 1;
data_Archive = [];
tic1 = tic; 
for timeOfFlight = 10:-0.01:0.1
    data = Data_set(type,timeOfFlight);
     for run = 1:10
         ticrun = tic; 
         [distToBall,trajCalc] = attemptCatch(type,timeOfFlight); % Print and calculate distance to ball
         data.Data_raw(run) = distToBall;
         toc(ticrun)
     end
     data_Archive{1,n} = data;
     fprintf(" Run %d of 1000 Complete\n",n)
     n = n +1 ;
     toc(tic1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Iterate through path, calculating robot positions and try to catch the ball
function [distToBall,trajCalc] = attemptCatch(type,timeOfFlight)

    % Initial robot which will catch the ball. Parameters can be
        % implemented if desired
    catcher = BallCatcher();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create a trajectory
    trajCalc = initializeTrajectoryFast(catcher,timeOfFlight);
%     trajCalc = initializeTrajectory(catcher,degreeOfRandomness,timeOfFlight);

%     distToBall = 0;
%     ball_pos = [0, 0, 0];
%     return;

    t = 0;% Initialize time to 0

    % Get the initial ball position
    ball_pos = trajCalc.getBallPos(t);
    
    % Iterate through positions until ball passes target
        % Needs to cover edge case when arm continues traveling past when ball interests workspace
    while(ball_pos(1) < 0) 
        % Get current ball position
        ball_pos = trajCalc.getBallPos(t);

        % Get ball prediction
        [intersectTime, y, z] = trajCalc.predictParabolic(t); 
        ball_pred_pos = [0, y, z];

        % Calculate robot pos to catch ball
        [configSol, solTime] = catcher.calcRobotPos(type, ball_pred_pos);
        t = t + solTime/3; % Increment time by solution time
        configSol.JointPosition;
        catcher.getEndEffectorPos(configSol);

        % Move arm within desired constraints (joint velocity)
        newConfig = catcher.moveArm(configSol, solTime);

        % Display robot movement
        %catcher.showRobot(newConfig, ball_pred_pos, ball_pos);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate final position of robot arm
    EEPos = catcher.getEndEffectorPos(newConfig);
    
    % Calculate distance to the ball
    distToBall = norm(ball_pos - EEPos);
end


function trajCalc = initializeTrajectory(catcher,degreeOfRandomness,timeOfFlight)
    transform = getTransform(catcher.robot,catcher.firstConfig,'tool','base');
    baseball = Ball(1.49, .375); %mass(kg),radius(meters)
    trajCalc = Trajectory_Calculator(baseball, 0, .01);
    %timeOfFlight = .2;%sec
    %degreeOfRandomness = .8;%meters
    initPos = [-18.44, 0.5645, 0.8448];%meters
    workspaceCenter = [0, 0, 0];%meters
    workspaceRadius = 0.8;%meters
    distanceThrown = 10;

    % generate values to fit within given parameters
%     [windVel, initVel, actualError, actualTimeOfFlight, initPos] = trajCalc.generateParameters(timeOfFlight, degreeOfRandomness, initPos, workspaceCenter, workspaceRadius);
    
% WORK IN PROGRESS, MEETS SPECIFICAITONS NOT RANDOM ENOUGH
    [windVel, initVel, ~, ~, initPos] = trajCalc.generateParameters2(workspaceCenter, degreeOfRandomness, timeOfFlight, distanceThrown);

%     % Hard coded, non random, for testing only
%     initPos = [-10, 0, 1];
%     initVel = [50, 0, 1];
%     windVel = [0, 0, 0];
% 
%     % Calculate trajectory and store full trajectory in traj
    traj = trajCalc.calcTrajectory(initPos, initVel, windVel);
    traj(end, 2:4);
end

function trajCalc = initializeTrajectoryFast(catcher,timeOfFlight)
    transform = getTransform(catcher.robot,catcher.firstConfig,'tool','base');
    baseball = Ball(.149, .0375); %mass(kg),radius(meters)
    trajCalc = Trajectory_Calculator(baseball, 0, .01);
    initPos = [-18.44, 0.5645, 0.8448];%meters
    workspaceCenter = [0, 0, 0];%meters
    workspaceRadius = 0.8;%meters
    distanceThrown = 10;

    [windVel, initVel, deviation, ~, initPos] = trajCalc.generateParameters3(workspaceCenter, distanceThrown, timeOfFlight);
    deviation

    traj = trajCalc.calcTrajectory(initPos, initVel, windVel);
    traj(end, 2:4);
end