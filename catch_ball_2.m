clc
clear
clf
close

%% Test/Calibrate Trajectory Generation
%Used only for testing/calibration
if false
    for i = .1:.01:.9
        [catcher, trajCalc, deviation, actualTimeOfFlight] = initializeTrajectoryFast(i);
        deviation
%         trajCalc.getBallPos(actualTimeOfFlight)
%         [~, y, z] = trajCalc.predictParabolic(0)
    end
    return
end

%% Data Collection
% Core section for gathering data
try
    % Attempt to load previous data
    load("ik_data2.mat") %Make unique name to run regardless of prior data
    load("anfis_data2.mat")
catch
    % Initialize data lists
    ikData = [];
    anfisData = [];
    
    % Iterate through desired range of flight times
    for timeOfFlight = .1:0.005:.9
        % Collect 15 data points for each flight time
        for run = 1:15
            % Initialize the trajectory
            [catcher, trajCalc, deviation, actualTimeOfFlight] = initializeTrajectoryFast(timeOfFlight);
            % Calculate robot's distance to the ball
            [distToBallIK, calcTimeIK] = attemptCatch(trajCalc, catcher, "ik");
            [distToBallANFIS, calcTimeANFIS] = attemptCatch(trajCalc, catcher, "anfis");
            %Add data to matrix
            ikData(end+1, :) = [actualTimeOfFlight, deviation, distToBallIK, calcTimeIK];
            anfisData(end+1, :) = [actualTimeOfFlight, deviation, distToBallANFIS, calcTimeANFIS];
        end
    end
    %Save data
    save("ik_data2.mat", "ikData")
    save("anfis_data2.mat", "anfisData")
end
Data_Visualization


%% Iterate through path, calculating robot positions and try to catch the ball
function [distToBall, calcTime] = attemptCatch(trajCalc, catcher, algo)
    t = 0;% Initialize time to 0
    % Get the initial ball position
    ball_pos = trajCalc.getBallPos(t);
    % Initialize empty array for calculation times
    calcTimes = [];
    algo; %if desired, print the algorithm for troubleshooting
    
    % Iterate through positions until ball passes target
    while(ball_pos(1) < 0)
        % Get current ball position
        ball_pos = trajCalc.getBallPos(t);

        % Get ball prediction
        [intersectTime, y, z] = trajCalc.predictParabolic(t);
        ball_pred_pos = [0, y, z];

        % Calculate robot pos to catch ball
        [configSol, solTime] = catcher.calcRobotPos(algo, ball_pred_pos);

        % Increment time by solution time
        t = t + solTime/3; 
        calcTimes(end + 1) = solTime;

        % Move arm within desired constraints (joint velocity)
%         newConfig = configSol; %comment next line to neglect velocity of arm
        newConfig = catcher.moveArm(configSol, solTime);

        % Display robot movement (warning: slow)
%        catcher.showRobot(newConfig, ball_pred_pos, ball_pos);
    end
    % Calculate final position of robot arm
    EEPos = catcher.getEndEffectorPos(newConfig);

    % Calculate distance to the ball
    distToBall = norm(ball_pos - EEPos);
    calcTime = mean(calcTimes);
end

function [catcher, trajCalc, deviation, actualTimeOfFlight] = initializeTrajectoryFast(timeOfFlight)
    % Initial robot which will catch the ball. Parameters can be
    % adjusted if desired
    catcher = BallCatcher();
    
    baseball = Ball(2, 0.1); %mass(kg),radius(meters)
    trajCalc = Trajectory_Calculator(baseball, 0, .0005); %Calculate trajectory
    initPredPos = [0, 0, 0.4]; %meters, where first prediction will lie
    distanceThrown = 10; %meters, normal distance from workspace to initial ball position
    
    [windVel, initVel, deviation, actualTimeOfFlight, initPos] = trajCalc.generateParameters3(initPredPos, distanceThrown, timeOfFlight);
    deviation; %Print deviation if needed for calibrating trajectory parameters
    
    % Calculate ball trajectory
    traj = trajCalc.calcTrajectory(initPos, initVel, windVel);
end