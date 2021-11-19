classdef Trajectory_Calculator < handle
    properties
        timestep
        traj
        ball
        targetX
    end
    methods
        function self = Trajectory_Calculator(ball, targetX, timestep)
            self.ball = ball;
            self.targetX = targetX;
            self.timestep = timestep;
            self.traj = [];
        end
        function F = calcNetForce(self, ballVel, windVel)
            g = [0, 0, -9.81]; %m / s^2
            airResistance = self.ball.calculateDrag(ballVel, windVel);
            F = airResistance + self.ball.mass * g;
        end

        function kin = updateKinematics(self, lastTraj, windVel, timestep)
            if ~exist('timestep','var')
                timestep = self.timestep;
            end
            forces = self.calcNetForce(lastTraj(5:7), windVel);
            acc = forces / self.ball.mass;
            velChange = acc * timestep;
            newVel = lastTraj(5:7) + velChange;
            posChange = newVel * timestep;
            newPos = lastTraj(2:4) + posChange;
            newTime = lastTraj(1) + timestep;
            kin = [newTime, newPos, newVel];
        end

        function traj = calcTrajectory(self, initPos, initVel, windVel)
            self.traj = [0, initPos, initVel]; % time, x pos, y pos, z pos, x vel, y vel, z vel
            %                                      1     2       3      4     5      6      7
            while self.traj(end, 2) < self.targetX % Repeat until desired position reached
                self.traj(end + 1, :) = self.updateKinematics(self.traj(end, :), windVel);
            end
            traj = self.traj;
        end

        function [intersectTime, y, z] = predictParabolic(self, time)
            g = -9.81;
            index = round(time / self.timestep) + 1;
            if index > length(self.traj)
                intersectTime = self.traj(end, 1);
                y = self.traj(end, 3);
                z = self.traj(end, 4);
                return;
            end
            lastTraj = self.traj(index, :);
            vel = lastTraj(5:7);
            pos = lastTraj(2:4);
            distToGo = self.targetX - pos(1);
            timeToGo = distToGo / vel(1);
            deltaY = vel(2) * timeToGo;
            deltaZ = vel(3) * timeToGo + 1/2 * g * timeToGo^2;
            intersectTime = time + timeToGo;
            y = pos(2) + deltaY;
            z = pos(3) + deltaZ;
        end

        function [intersectTime, y, z] = predictLinear(self, time)
            index = round(time / self.timestep) + 1;
            if index > length(self.traj)
                intersectTime = self.traj(end, 1);
                y = self.traj(end, 3);
                z = self.traj(end, 4);
                return;
            end
            lastTraj = self.traj(index, :);
            vel = lastTraj(5:7);
            pos = lastTraj(2:4);
            distToGo = self.targetX - pos(1);
            timeToGo = distToGo / vel(1);
            deltaY = vel(2) * timeToGo;
            deltaZ = vel(3) * timeToGo;
            intersectTime = time + timeToGo;
            y = pos(2) + deltaY;
            z = pos(3) + deltaZ;
        end

        function pos = getBallPos (self, time)
            index = floor(time / self.timestep + 1);
            if index > length(self.traj)
                pos = self.traj(end, 2:4);
                return;
            end
            remainder = mod(time, self.timestep);
            kin = self.updateKinematics(self.traj(index, :), remainder);
            pos = kin(2:4);
        end

        function [newTimeOfFlightError, newError, actualTimeOfFlight, actualEndPos] = checkParameters(self, ballVelGuess, windVelGuess, initPos, targetTimeOfFlight)
            initTraj = calcTrajectory(self, initPos, ballVelGuess, windVelGuess);

            [~, y, z] = self.predictParabolic(0);
            initPredPos = [1, y, z];
            actualEnd = initTraj(end, :);
            actualEndPos = actualEnd(2:4);
            actualErrorVec = actualEndPos - initPredPos;
            newError = abs(norm(actualErrorVec));
            if actualEndPos(2) < initPredPos(2)
                newError = newError * -1;
            end

            actualTimeOfFlight = actualEnd(1);
            newTimeOfFlightError = actualTimeOfFlight - targetTimeOfFlight;
        end

        function [windVel, initVel, actualError, actualTimeOfFlight, initPos] = generateParameters(self, timeOfFlight, degreeOfRandomness, initPos, workspaceCenter, workspaceRadius, tolerance)
            if ~exist('tolerance','var')
                tolerance = 0.001;
            end
            xVelGuess = ((workspaceCenter(1) - initPos(1))) / timeOfFlight;
            yVelGuess = 0;
            syms initZVel
            zVelGuess = double(vpa(solve(workspaceCenter(3) == initZVel * timeOfFlight - 9.81 / 2 * timeOfFlight^2)));

            windXVelGuess = 0;
            windYVelGuess = degreeOfRandomness / timeOfFlight;
            windZVelGuess = 0;

            initVel = [xVelGuess, yVelGuess, zVelGuess];
            windVel = [windXVelGuess, windYVelGuess, windZVelGuess];
            
            [newTimeOfFlightError, actualError, actualTimeOfFlight] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);

            while abs(newTimeOfFlightError) > tolerance
                initVel(1) = initVel(1) * (1 + newTimeOfFlightError / actualTimeOfFlight);
                [newTimeOfFlightError, actualError, actualTimeOfFlight, actualEndPos] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
            end
            
            maxAttempts = 1000;
            
            while abs(actualError - degreeOfRandomness) > tolerance && maxAttempts > 0
                windVel(2) = windVel(2) + (degreeOfRandomness - actualError);
                [newTimeOfFlightError, actualError, actualTimeOfFlight, actualEndPos] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
                maxAttempts = maxAttempts - 1;
            end
            if norm(actualEndPos) > workspaceRadius
                initPos = initPos - actualEndPos;
                initPos(1) = (.5 - rand()) * workspaceRadius;
                initPos(3) = (.5 - rand()) * workspaceRadius;
            end

            if rand() > 0.5
                windVel(2) = -windVel(2);
            end
        end
    end
end