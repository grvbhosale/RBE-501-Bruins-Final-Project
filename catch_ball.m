clc
clear
clf

distToBall = attempCatch() % Print and calculate distance to ball

% Iterate through path, calculating robot positions and try to catch the ball
function distToBall = attempCatch()

    % Initial robot which will catch the ball. Parameters can be
        % implemented if desired
    robot = BallCatcher();

    % Create a trajectory
    trajCalc = initializeTrajectory();

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
        [configSol, solTime] = robot.calcRobotPos("ik", ball_pred_pos);
        t = t + solTime; % Increment time by solution time

        % Move arm within desired constraints (joint velocity)
        newConfig = robot.moveArm(configSol, solTime);

        % Display robot movement
        robot.showRobot(newConfig, ball_pred_pos, ball_pos);
    end

    % Calculate final position of robot arm
    EEPos = robot.getEndEffectorPos(newConfig);
    
    % Calculate distance to the ball
    distToBall = norm(ball_pos - EEPos);
end


function trajCalc = initializeTrajectory()
    baseball = Ball(.6, .06); %mass(kg),radius(meters)
    trajCalc = Trajectory_Calculator(baseball, 0, .001);
    timeOfFlight = 1;%sec
    degreeOfRandomness = 2;%meters
    initPos = [-20, 0, 1];%meters
    workspaceCenter = [0, 0, 0];%meters
    workspaceRadius = 1.5;%meters

    % generate values to fit within given parameters
    [windVel, initVel, actualError, actualTimeOfFlight, initPos] = trajCalc.generateParameters(timeOfFlight, degreeOfRandomness, initPos, workspaceCenter, workspaceRadius);


    % Hard coded, non random, for testing only
    initPos = [-18.44, -4, -15];
    initVel = [10, 0, 15];
    windVel = [5, 6, 0];

    % Calculate trajectory and store full trajectory in traj
    traj = trajCalc.calcTrajectory(initPos, initVel, windVel);

end