%% Scatter Plots
% Initialize catch conditions
cutoffDist = .1;
failureDist = .5;
dataPoints = size(ikData);
dataPoints = dataPoints(1);

% Initialize catch type matrices
ikDataCatch = [];
ikDataNearMiss = [];
ikDataError = [];
anfisDataCatch = [];
anfisDataNearMiss = [];
anfisDataError = [];

% Catagorize each throw
for i = 1:dataPoints
    if ikData(i,3) < cutoffDist
        ikDataCatch(end + 1, :) = ikData(i,1:2);
    elseif ikData(i,3) > failureDist
        ikDataError(end + 1, :) = ikData(i,1:2);
    else
        ikDataNearMiss(end + 1, :) = ikData(i,1:2);
    end
    if anfisData(i,3) < cutoffDist
        anfisDataCatch(end + 1, :) = anfisData(i,1:2);
    elseif anfisData(i,3) > failureDist
        anfisDataError(end + 1, :) = anfisData(i,1:2);
    else
        anfisDataNearMiss(end + 1, :) = anfisData(i,1:2);
    end
end
% Print catch percent
disp(length(ikDataCatch) / length(ikData)*100)
disp(length(anfisDataCatch) / length(anfisData)*100)

% Display IK scatter plot
figure(1)
scatter(ikDataCatch(:,1), ikDataCatch(:,2), 10, "green", 'filled')
hold on
scatter(ikDataNearMiss(:,1), ikDataNearMiss(:,2), 10, "red", 'filled')
try
    scatter(ikDataError(:,1), ikDataError(:,2), 10, "black", 'filled')
catch
    scatter([.1], [0], .1, "black", 'filled')
end
hold off
xlabel("Time Of Flight (s)")
ylabel("Deviation (m)")
title("IK Data")
legend(["Catch", "Near Catch", "Miss"], "Location","best")

% Display ANFIS scatter plot
figure(2)
hold on
scatter(anfisDataCatch(:,1), anfisDataCatch(:,2), 10, "green", 'filled')
scatter(anfisDataNearMiss(:,1), anfisDataNearMiss(:,2), 10, "red", 'filled')
scatter(anfisDataError(:,1), anfisDataError(:,2), 10, "black", 'filled')
hold off
xlabel("Time Of Flight (s)")
ylabel("Deviation (m)")
title("ANFIS Data")
legend(["Miss", "Catch", "Near Catch"], "Location","best")

%% 2D Line Plots
% Repeated cycles at each time interval
dataReps = 15; % make sure this is set properly!

% Initialize timestep-averaged data matricies
ikDataAverage = [];
anfisDataAverage = [];

% Find the average of each trajectory at the each timestep
% Iterate through each set of timesteps
for i = 1:dataReps:dataPoints
    % Initialize sums
    sumIK = 0;
    sumANFIS = 0;
    sumTime = 0;
    sumDev = 0;
    sumCalcTIK = 0;
    sumCalcTAFNIS = 0;

    % Iterate through each datapoint with the same timestep
    for j = 1:dataReps-1
        sumIK = sumIK + ikData(i+j,3);
        sumANFIS = sumANFIS + anfisData(i+j,3);
        sumTime = sumTime() + ikData(i+j,1);
        sumDev = sumDev() + ikData(i+j,2);
        sumCalcTIK = sumCalcTIK() + ikData(i+j,4); %not in full dataset currently
        sumCalcTAFNIS = sumCalcTAFNIS() + anfisData(i+j,4); %not in full dataset currently
    end

    % Average the data
    ikDataAverage(end + 1, :) = [sumTime, sumDev, sumIK, sumCalcTIK] / dataReps;
    anfisDataAverage(end + 1, :) = [sumTime, sumDev, sumANFIS, sumCalcTAFNIS] / dataReps;
end

% Plot average error vs time of flight
figure(3)
plot(ikDataAverage(:,1), ikDataAverage(:,3))
hold on
plot(anfisDataAverage(:,1), anfisDataAverage(:,3))
hold off
xlabel("Time Of Flight (s)")
ylabel("Error between Robot and Ball (m)")
title("Comparing Error with Time of Flight")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','best')

% Plot average error vs deviation
% Sort data by deviation
ikDataAverage = sortrows(ikDataAverage, 2);
anfisDataAverage = sortrows(anfisDataAverage, 2);
figure(4)
plot(ikDataAverage(:,2), ikDataAverage(:,3))
hold on
plot(anfisDataAverage(:,2), anfisDataAverage(:,3))
hold off
xlabel("Deviation from Predicted Path (m)")
ylabel("Error between Robot and Ball (m)")
title("Comparing Error with Deviation")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','best')

%% Plot Average Calculation Time
% Resort data by time
ikDataAverage = sortrows(ikDataAverage, 1);
anfisDataAverage = sortrows(anfisDataAverage, 1);

%Keep only unique data poins
[~,uidx] = unique(ikDataAverage(:,1),'stable');
ikDataAverage = ikDataAverage(uidx,:);
[~,uidx] = unique(anfisDataAverage(:,1),'stable');
anfisDataAverage = anfisDataAverage(uidx,:);

% Display plot of computation time vs time of flight
figure(5)
bar(ikDataAverage(:,1), ikDataAverage(:,4), 'EdgeColor', [0 .9 .9], 'LineWidth',1.5)
hold on
bar(anfisDataAverage(:,1), anfisDataAverage(:,4), 'EdgeColor', [.9 0 0], 'LineWidth',1.5)
hold off
xlabel("Time Of Flight (s)")
ylabel("Average Computation Time (m)")
title("Comparing Computation Time with Time of Flight")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','northwest')

% Display plot of computation time vs deviation
figure(6)
bar(ikDataAverage(:,2), ikDataAverage(:,4), 'EdgeColor', [0 .9 .9], 'LineWidth',1.5)
hold on
bar(anfisDataAverage(:,2), anfisDataAverage(:,4), 'EdgeColor', [.9 0 0], 'LineWidth',1.5)
hold off
xlabel("Deviation from Intially Predicted Position (m)")
ylabel("Average Computation Time (m)")
title("Comparing Computation Time with Deviation")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','northwest')