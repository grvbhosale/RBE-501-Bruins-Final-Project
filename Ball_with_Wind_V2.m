clc
clear
close all

baseball = Ball(.6, .06);
traj = calcTrajectory(baseball, [5, 6, 0], .001, [-40, -4, 1], [10, 0, 15], 0);

plot3(traj(:, 2), traj(:, 3), traj(:, 4))

hold on
timeStep = .01;
timeCount = .001;
while 1
    try
        [t, y, z] = predictParabolic(traj, 0, timeCount, .001);
        hold on
        plot3(0, y, z, '*')
        hold off

        [t, y, z] = predictLinear(traj, 0, timeCount, .001);
        hold on
        plot3(0, y, z, '*')
        hold off

        timeCount = timeCount + timeStep;
    catch e
        e
        pause(.1);
        return
    end
end
% figure
% plot(traj(:, 1), traj(:, 2), traj(:, 1), traj(:, 3), traj(:, 1), traj(:, 4))

function F = calcNetForce(ball, ballVel, windVel) 
    g = [0, 0, -9.81]; %m / s^2

    airResistance = ball.calculateDrag(ballVel, windVel);

    F = airResistance + ball.mass * g;
end

function kin = updateKinematics(ball, lastTraj, timestep, windVel)
    forces = calcNetForce(ball, lastTraj(5:7), windVel);
    acc = forces / ball.mass;
    velChange = acc * timestep;
    newVel = lastTraj(5:7) + velChange;
    posChange = newVel * timestep;
    newPos = lastTraj(2:4) + posChange;
    newTime = lastTraj(1) + timestep;
    kin = [newTime, newPos, newVel];
end

function traj = calcTrajectory(ball, windVel, timeStep, initPos, initVel, targetX)
    traj = [0, initPos, initVel]; % time, x pos, y pos, z pos, x vel, y vel, z vel
    %                                 1     2       3      4     5      6      7
    while traj(end, 2) < targetX % Repeat until desired position reached
        traj(end + 1, :) = updateKinematics(ball, traj(end, :), timeStep, windVel);
    end
end

function [intersectTime, y, z] = predictParabolic(traj, targetX, time, timestep)
    g = -9.81;
    index = round(time / timestep);
    lastTraj = traj(index, :);
    vel = lastTraj(5:7);
    pos = lastTraj(2:4);
    distToGo = targetX - pos(1);
    timeToGo = distToGo / vel(1);
    deltaY = vel(2) * timeToGo;
    deltaZ = vel(3) * timeToGo + 1/2 * g * timeToGo^2;
    intersectTime = time + timeToGo;
    y = pos(2) + deltaY;
    z = pos(3) + deltaZ;
end

function [intersectTime, y, z] = predictLinear(traj, targetX, time, timestep)
    g = -9.81;
    index = round(time / timestep);
    lastTraj = traj(index, :);
    vel = lastTraj(5:7);
    pos = lastTraj(2:4);
    distToGo = targetX - pos(1);
    timeToGo = distToGo / vel(1);
    deltaY = vel(2) * timeToGo;
    deltaZ = vel(3) * timeToGo;
    intersectTime = time + timeToGo;
    y = pos(2) + deltaY;
    z = pos(3) + deltaZ;
end

function [x, y, z] = getBallPos (traj, trajectoryTimestep, time)
    index = time / trajectoryTimestep;
    remainder = mod(time, trajectoryTimestep);
end
