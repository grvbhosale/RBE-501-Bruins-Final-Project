clc
clear
clf
close

%% Test/Calibrate Trajectory Generation
if false
    catcher = BallCatcher(type);
    ticTest = tic;
    ticTest2 = tic;
    for i = .1:.01:1
        initializeTrajectoryFast(catcher, i);
        if(rem(i,1) == 0)
            timeInt = toc(ticTest2);
            fprintf("%d, Average Time: %d\n", i, timeInt/100)
            ticTest2 = tic;
        end
    end
    toc(ticTest)
    return
end

%% Data Collection
% Core section for gathering data this section will record the initial
% parameters based on 
n = 1;
data_Archive = [];

%storing these for plotting
flighttime = [];
mean_ik = [];
mean_anfis = [];
difference = [];
deviation_all = [];
distance_ik_all = [];
distance_anfis_all = [];
ft_all = [];
tic1 = tic; 
for timeOfFlight = 1:-0.01:0.1
    data = Data_set(timeOfFlight);
     for run = 1:10
         ticrun = tic; 
         % Print and calculate distance to ball
         [data,trajCalc] = attemptCatch(timeOfFlight,data,run);
         toc(ticrun)
     end
%      data.Mean_ik = mean(data.Data_raw(1,:));
     data.Mean_anfis = mean(data.Data_raw(2,:));
     data_Archive{1,n} = data;
     fprintf(" Run %d of 91 Complete\n",n)
     %storing these for plotting
     flighttime = [flighttime,timeOfFlight];
%      mean_ik = [mean_ik,data.Mean_ik];
     mean_anfis = [mean_anfis,data.Mean_anfis];
%      difference = [difference, abs(data.Mean_ik-data.Mean_anfis)];
     deviation_all = [deviation_all,data.Deviation];
%      distance_ik_all = [distance_ik_all,data.Data_raw(1,:)];
     distance_anfis_all = [distance_anfis_all,data.Data_raw(2,:)];
     for i = 1:10
        ft_all = [ft_all,data.TimeOfFlight];
     end
     n = n +1 ;
     toc(tic1)
end


%% Iterate through path, calculating robot positions and try to catch the ball
function [data,trajCalc] = attemptCatch(timeOfFlight,data,run)

    % Initial robot which will catch the ball. Parameters can be
        % implemented if desired
    catcher = BallCatcher();
%     catcher_ik = catcher;
    catcher_anfis = catcher;

    % Create a trajectory
    [initPos,initVel, deviation,trajCalc] = initializeTrajectoryFast(catcher,timeOfFlight);
%     trajCalc = initializeTrajectory(catcher,degreeOfRandomness,timeOfFlight);
    data.InitialPosition(1,run) = initPos(1); %x
    data.InitialPosition(2,run) = initPos(2); %y
    data.InitialPosition(3,run) = initPos(3); %z
    data.InitialVelocity(1,run) = initVel(1); %v_x
    data.InitialVelocity(2,run) = initVel(2); %v_y
    data.InitialVelocity(3,run) = initVel(3); %v_z
    data.Deviation(run) = deviation;
    deviation;
%     distToBall = 0;
%     ball_pos = [0, 0, 0];
%     return;

    t = 0;% Initialize time to 0
    t_ik = 0;
    t_anfis = 0;
    itr_time_list_ik = [];
    itr_time_list_anfis = [];
    % Get the initial ball position
    ball_pos = trajCalc.getBallPos(t);
    h = figure;
    filename = 'testAnimated.gif';
    m=1;
    % Iterate through positions until ball passes target
        % Needs to cover edge case when arm continues traveling past when ball interests workspace
    while(ball_pos(1) < 0) 
        % Get current ball position
        
%         t = t_ik;
%         ball_pos = trajCalc.getBallPos(t);
%         if ball_pos(1) < 0
%             % Get ball prediction
%             [intersectTime, y, z] = trajCalc.predictParabolic(t); 
%             ball_pred_pos = [0, y, z];
% 
%             % Calculate robot pos to catch ball
%             [configSol_ik, solTime_ik] = catcher_ik.calcRobotPos("ik", ball_pred_pos);
%             itr_time_list_ik = [itr_time_list_ik,solTime_ik];
%             t_ik = t + solTime_ik/3; % Increment time by solution time
% %           configSol.JointPosition;
% %           catcher.getEndEffectorPos(configSol);
%         end
        % Get current ball position
        t = t_anfis;
        ball_pos = trajCalc.getBallPos(t);
        if ball_pos(1) < 0
            % Get ball prediction
            [intersectTime, y, z] = trajCalc.predictParabolic(t); 
            ball_pred_pos = [0, y, z];
        
            % Calculate robot pos to catch ball
            [configSol_anfis, solTime_anfis] = catcher_anfis.calcRobotPos("anfis", ball_pred_pos);
            itr_time_list_anfis = [itr_time_list_anfis,solTime_anfis];
            t_anfis = t + solTime_anfis; % Increment time by solution time
%           configSol.JointPosition;
%           catcher.getEndEffectorPos(configSol);
        end
        % Move arm within desired constraints (joint velocity)
%         newConfig_ik = catcher_ik.moveArm(configSol_ik, solTime_ik);
        newConfig_anfis = catcher_anfis.moveArm(configSol_anfis, solTime_anfis);
        %Display robot movement
        catcher.showRobot(newConfig_anfis, ball_pred_pos, ball_pos);
        ball_pos = trajCalc.getBallPos(t);
    end
    
    % Calculate final position of robot arm
%     EEPos_ik = catcher_ik.getEndEffectorPos(newConfig_ik);
    EEPos_anfis = catcher_anfis.getEndEffectorPos(newConfig_anfis);
    % Calculate distance to the ball
%     distToBall_ik = norm(ball_pos - EEPos_ik);
    distToBall_anfis = norm(ball_pos - EEPos_anfis);
    
%     mean_itr_time_ik = mean(itr_time_list_ik);
    mean_itr_time_anfis = mean(itr_time_list_anfis);
    
%     data.Data_raw(1,run) = distToBall_ik;
    data.Data_raw(2,run) = distToBall_anfis;
%     data.Interval_ik(1,run) = mean_itr_time_ik;
    data.Interval_anfis(1,run) = mean_itr_time_anfis;
end


function trajCalc = initializeTrajectory(catcher,degreeOfRandomness,timeOfFlight)
    transform = getTransform(catcher.robot,catcher.firstConfig,'tool','base');
    baseball = Ball(1.49, .375); %mass(kg),radius(meters)
    trajCalc = Trajectory_Calculator(baseball, 0, .01);
    %timeOfFlight = .2;%sec
    %degreeOfRandomness = .8;%meters
    initPos = [-18.44, 0.5645, 0.8448];%meters
    workspaceCenter = [0, 0, 0.8448];%meters
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

function [initPos,initVel, deviation,trajCalc] = initializeTrajectoryFast(catcher,timeOfFlight)
    transform = getTransform(catcher.robot,catcher.firstConfig,'tool','base');
    baseball = Ball(1, 0.17); %mass(kg),radius(meters)
    trajCalc = Trajectory_Calculator(baseball, 0, .0005);
    workspaceCenter = [0, 0, 0.8448];%meters
    distanceThrown = 10;

    [windVel, initVel, deviation, actualTimeOfFlight, initPos] = trajCalc.generateParameters3(workspaceCenter, distanceThrown, timeOfFlight);
    deviation;
    
    traj = trajCalc.calcTrajectory(initPos, initVel, windVel);
%     [~, ~, zPredFirst] = trajCalc.predictParabolic(0)
%     traj(end, 2:4)
end