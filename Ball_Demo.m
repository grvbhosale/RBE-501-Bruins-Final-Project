clc
clear
clf
pause(.01)
tic;

ballMass = .5;
ballRadius = .05;
baseball = Ball(ballMass, ballRadius);

robotX = 0;
windVel = [0, 0, 0];
timestep = 0.0001;


trajCalc = Trajectory_Calculator(baseball, robotX, timestep);

initPos = [-20, 0, 1];
initVel = [30, 0, 0];
traj = trajCalc.calcTrajectory(initPos, initVel, windVel);

toc

plot3(traj(:, 2), traj(:, 3), traj(:, 4),'LineWidth',5)

timeStep = .01;
timeCount = 0;

lastTime = -1;
t = 0;
ylim([-1 1])
while t ~= lastTime
    lastTime = t;
    [t, y, z] = trajCalc.predictParabolic(timeCount);
    t;
    startPos = trajCalc.getBallPos(timeCount);
    xs = [0, startPos(1)];
    ys = [y, startPos(2)];
    zs = [z, startPos(3)];
    hold on
    plot3(xs, ys, zs)
    hold off
    timeCount = timeCount + timeStep;
    pause(.003)
end

timeStep = .5;
timeCount = .001;

lastTime = -1;
t = 0;
while t ~= lastTime
    lastTime = t;
    [t, y, z] = trajCalc.predictParabolic(timeCount);
    startPos = trajCalc.getBallPos(timeCount);
    xs = [0, startPos(1)];
    ys = [y, startPos(2)];
    zs = [z, startPos(3)];
    hold on
    plot3(xs, ys, zs,'LineWidth', 4)
    hold off
    timeCount = timeCount + timeStep;
    pause(.1)
end