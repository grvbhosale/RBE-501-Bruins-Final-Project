% Scatter Plot
cutoffDist = .15;
failureDist = .5;
dataPoints = size(ikData);
dataPoints = dataPoints(1);

ikDataCatch = [];
ikDataNearMiss = [];
ikDataError = [];
anfisDataCatch = [];
anfisDataNearMiss = [];
anfisDataError = [];
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
disp(length(ikDataCatch) / length(ikData)*100)
disp(length(anfisDataCatch) / length(anfisData)*100)

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

% Plot
dataReps = 15;
ikDataAverage = [];
anfisDataAverage = [];
for i = 1:dataReps:dataPoints
    sumIK = 0;
    sumANFIS = 0;
    sumTime = 0;
    sumDev = 0;
    sumCalcTIK = 0;
    sumCalcTAFNIS = 0;
    for j = 1:dataReps-1
        sumIK = sumIK + ikData(i+j,3);
        sumANFIS = sumANFIS + anfisData(i+j,3);
        sumTime = sumTime() + ikData(i+j,1);
        sumDev = sumDev() + ikData(i+j,2);
        sumCalcTIK = sumCalcTIK() + ikData(i+j,4); %not in full dataset currently
        sumCalcTAFNIS = sumCalcTAFNIS() + anfisData(i+j,4); %not in full dataset currently
    end
    ikDataAverage(end + 1, :) = [sumTime, sumDev, sumIK, sumCalcTIK] / dataReps;
    anfisDataAverage(end + 1, :) = [sumTime, sumDev, sumANFIS, sumCalcTAFNIS] / dataReps;
end

figure(3)
plot(ikDataAverage(:,1), ikDataAverage(:,3))
hold on
plot(anfisDataAverage(:,1), anfisDataAverage(:,3))
hold off
xlabel("Time Of Flight (s)")
ylabel("Error between Robot and Ball (m)")
title("Comparing Error with Time of Flight")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','best')

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

% Line Plot
ikDataAverage = sortrows(ikDataAverage, 1);
anfisDataAverage = sortrows(anfisDataAverage, 1);

[~,uidx] = unique(ikDataAverage(:,1),'stable');
ikDataAverage = ikDataAverage(uidx,:);

[~,uidx] = unique(anfisDataAverage(:,1),'stable');
anfisDataAverage = anfisDataAverage(uidx,:);

figure(5)
bar(ikDataAverage(:,1), ikDataAverage(:,4), 'EdgeColor', [0 .9 .9], 'LineWidth',1.5)
hold on
bar(anfisDataAverage(:,1), anfisDataAverage(:,4), 'EdgeColor', [.9 0 0], 'LineWidth',1.5)
hold off
xlabel("Time Of Flight (s)")
ylabel("Average Computation Time (m)")
title("Comparing Computation Time with Time of Flight")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','northwest')

figure(6)
bar(ikDataAverage(:,2), ikDataAverage(:,4), 'EdgeColor', [0 .9 .9], 'LineWidth',1.5)
hold on
bar(anfisDataAverage(:,2), anfisDataAverage(:,4), 'EdgeColor', [.9 0 0], 'LineWidth',1.5)
hold off
xlabel("Deviation from Intially Predicted Position (m)")
ylabel("Average Computation Time (m)")
title("Comparing Computation Time with Deviation")
legend({"Inverse Kinematics", "ANFIS"}, 'Location','northwest')