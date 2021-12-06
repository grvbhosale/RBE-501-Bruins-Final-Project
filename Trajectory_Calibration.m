%% Test/Calibrate Trajectory Generation
%Used only for testing
ticTest = tic;
ticTest2 = tic;
for i = .1:.01:.9
    [catcher, trajCalc, deviation, actualTimeOfFlight] = initializeTrajectoryFast(i);
    deviation
%         trajCalc.getBallPos(actualTimeOfFlight)
%         [~, y, z] = trajCalc.predictParabolic(0)
end
toc(ticTest)
return
