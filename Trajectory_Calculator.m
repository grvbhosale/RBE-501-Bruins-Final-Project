classdef Trajectory_Calculator < handle
    properties
        timestep
        traj
        ball
        targetX
    end
    methods
        % Initialze trajectory object
        function self = Trajectory_Calculator(ball, targetX, timestep)
            self.ball = ball;
            self.targetX = targetX;
            self.timestep = timestep;
            self.traj = [];
        end
        % Calculate net force on ball
        function F = calcNetForce(self, ballVel, windVel)
            g = [0, 0, -9.81]; %m / s^2
            % Get drag from ball
            airResistance = self.ball.calculateDrag(ballVel, windVel);
            % Sum forces
            F = airResistance + self.ball.mass * g;
        end
        % Solve for the new time, position, and velocity of the ball
        function kin = updateKinematics(self, lastTraj, windVel, timestep)
            % Use default timestep if not defined
            if ~exist('timestep','var')
                timestep = self.timestep;
            end
            % Calculate the force on the ball
            forces = self.calcNetForce(lastTraj(5:7), windVel);
            % From the force, calculate the acceleration
            acc = forces / self.ball.mass;
            % From the acceleration, calculate the change in velocity
            velChange = acc * timestep;
            % From the delta velocity, find the new velocity
            newVel = lastTraj(5:7) + velChange;
            % From the new velocity, find the change in position
            posChange = newVel * timestep;
            % From the delta position, find the new position
            newPos = lastTraj(2:4) + posChange;
            % Increment the time by the timestep
            newTime = lastTraj(1) + timestep;
            % Combine all prior data into 1 matrix to return
            kin = [newTime, newPos, newVel];
        end
        % Calculate the full trajectory of the ball
        function traj = calcTrajectory(self, initPos, initVel, windVel)
            % Initialize the trajectory
            self.traj = [0, initPos, initVel]; % time, x pos, y pos, z pos, x vel, y vel, z vel
            %                                      1     2       3      4     5      6      7
            % Iterate through the trajectory until reaching the arm
            while self.traj(end, 2) < self.targetX
                % Add new kinematics to trajectory history
                self.traj(end + 1, :) = self.updateKinematics(self.traj(end, :), windVel);
            end
            % Return trajectory
            traj = self.traj;
        end
        % Predict where the ball would end up, neglecting air resistance
        function [intersectTime, y, z] = predictParabolic(self, time)
            g = -9.81; % m/s^2
            % Round to closest index
            index = round(time / self.timestep) + 1;
            % If asking for predicted position after the trajectory has been
            % completed, return the last position
            if index > length(self.traj)
                intersectTime = self.traj(end, 1);
                y = self.traj(end, 3);
                z = self.traj(end, 4);
                return;
            end
            % Get kinematics just before current time
            lastTraj = self.traj(index, :);
            % Isolate the position and velocity
            vel = lastTraj(5:7);
            pos = lastTraj(2:4);
            % Approximate distance to target
            distToGo = self.targetX - pos(1);
            % Approximate time to target
            timeToGo = distToGo / vel(1);
            % Calculate delta y position assuming no y acceleration
            deltaY = vel(2) * timeToGo;
            % calculate delta z position accounting for gravity
            deltaZ = vel(3) * timeToGo + 1/2 * g * timeToGo^2;
            % Estimate when ball will hit target
            intersectTime = time + timeToGo;
            % Calculate final position from delta position 
            y = pos(2) + deltaY;
            z = pos(3) + deltaZ;
        end
        % Predict where the ball would end up from a linear trajectory
            % Depricated
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
        % Get ball position at any point in time
        function pos = getBallPos (self, time)
            % Get index of previous time
            index = floor(time / self.timestep + 1);
            % If index after end of trajectory, use last position
            if index > length(self.traj)
                pos = self.traj(end, 2:4);
                return;
            end
            % Calculate the time from last time to requested time
            remainder = mod(time, self.timestep);
            % Get position from last known trajectory point
            kin = self.updateKinematics(self.traj(index, :), remainder);
            % Isolate position from trajectory
            pos = kin(2:4);
        end
        % Check the properties of the resulting trajectory to determine if
        % it is sufficiently meeting the desired parameters
        function [newTimeOfFlightError, newError, actualTimeOfFlight, actualEndPos] = checkParameters(self, ballVelGuess, windVelGuess, initPos, targetTimeOfFlight)
            % Calculate the trajectory
            initTraj = calcTrajectory(self, initPos, ballVelGuess, windVelGuess);
            % Get the initial predicted position
            [~, y, z] = self.predictParabolic(0);
            initPredPos = [0, y, z];
            % Get the actual final position
            actualEnd = initTraj(end, :);
            actualEndPos = actualEnd(2:4);
            % Calculate the deviation vector
            actualErrorVec = actualEndPos - initPredPos;
            % Get the magnitude of the error vector
            newError = abs(norm(actualErrorVec));
            % Use the sign of the error to indicate error direction in the
            % y direction
            if actualEndPos(2) < initPredPos(2)
                newError = newError * -1;
            end
            % Isolate time of flight
            actualTimeOfFlight = actualEnd(1);
            % Calculate error in the time of flight
            newTimeOfFlightError = actualTimeOfFlight - targetTimeOfFlight;
        end

        % First attempt at generating parameters. Error prone
        % Depricated
        function [windVel, initVel, actualError, actualTimeOfFlight, initPos] = generateParameters(self, timeOfFlight, degreeOfRandomness, initPos, workspaceCenter, workspaceRadius, initPredPos, tolerance)
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

        % Find velocity to achieve desired error. Too slow for practical use
        % Depricated
        function [minErrorZVel, minError] = findClosestVelocity(self, targetZError, initVel, timeOfFlight, initPos, minOrMax, maxTime, tolerance)
            tic;
            if ~exist('maxTime','var')
                maxTime = 5;
            end
            if ~exist('tolerance','var')
                tolerance = 0.001;
            end
            minErrorZVel = 0;
            initVel(3) = minErrorZVel;
            wind0 = [0, 0, 0];
            [~, minError, ~, ~] = self.checkParameters(initVel, wind0, initPos, timeOfFlight);
            minError = abs(targetZError) - minError;
            dataRange = 200;
            dataCenter = 0;
            dataStep = 1;

            
            while abs(minError) > tolerance && toc < maxTime
                abs(minError) > tolerance;
                toc < maxTime;
                dataStart = dataCenter - dataRange;
                dataStop = dataCenter + dataRange;
                zVels = dataStart:dataStep:dataStop;
                errors = [];
                for j = zVels
                    initVel(3) = j;
                    [~, ZVelError, ~, ~] = self.checkParameters(initVel, wind0, initPos, timeOfFlight);
                    ZVelError = abs(targetZError) - ZVelError;
                    errors(end+1, :) = [abs(ZVelError), j];
                end
%                 plot(errors(:, 2), errors(:, 1))
%                 toc
                pause(.01)
%                 if min(errors(:, 1)) > 0 || max(errors(:, 1)) < 0
%                     dataRange = dataRange * 10;
%                     dataStep = dataStep * 10;
%                     continue
%                 end

%                 minError = min(abs(errors(:, 1)));
%                 [row, ~, ~] = find(abs(errors(:, 1)) == minError);
%                 minErrorZVel = errors(row, 2);

                TF = islocalmin(errors(:, 1));
                minErrorIndex = find(TF, 1, minOrMax);
                dataRange = dataRange / 10;
                dataStep = dataStep / 10;
                minErrorZVel = errors(minErrorIndex, 2);
                dataCenter = minErrorZVel;
                minError = errors(minErrorIndex, 1);
            end
        end

        % Generates parameters meeting deviation and time of flight
        % requirements, but takes too long to complete
        % Depricated
        function [windVel, initVel, actualError, actualTimeOfFlight, initPos] = generateParameters2(self, initPredPos, deviation, timeOfFlight, distanceThrown, initRange, tolerance)
            % Initialize optional variables
            if ~exist('tolerance','var')
                tolerance = 0.001;
            end
            if ~exist('initRange','var')
                initRange = [2, -2; 1000, -1000]; %Ymax, Ymin, 
            end

            % Make initial velocity guesses
            xVelGuess = distanceThrown / timeOfFlight;
            yVelGuess = 0;
            syms initZVel
            zVelGuess = 0;
            % Parabolic to end at 0
%             zVelGuess = double(vpa(solve(0 == initZVel * timeOfFlight - 9.81 / 2 * timeOfFlight^2)));

            % Assume initial position
            initPos = [-distanceThrown, 0, 0];

            % Guess wind velocity
            windXVelGuess = 0;
            windYVelGuess = deviation / timeOfFlight;
            windZVelGuess = 0;

            % Initialize guess vectors
            initVel = [xVelGuess, yVelGuess, zVelGuess];
            windVel = [windXVelGuess, windYVelGuess, windZVelGuess];
            
            % Determine x velocity based on time of flight
            [newTimeOfFlightError, actualError, actualTimeOfFlight] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
            while abs(newTimeOfFlightError) > tolerance
                initVel(1) = initVel(1) * (1 + newTimeOfFlightError / actualTimeOfFlight);
                [newTimeOfFlightError, actualError, actualTimeOfFlight, actualEndPos] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
            end

            % Determine the z velocities for the min and max deviation
            [minErrorZVel, minError] = self.findClosestVelocity(0, initVel, timeOfFlight, initPos, "first", 5, .01);
            [minErrorZVel2, minError2] = self.findClosestVelocity(0, initVel, timeOfFlight, initPos, "last", 5, .01);
%             self.predictParabolic(0)
%             self.traj(end, :)
            [maxErrorZVel, maxError] = self.findClosestVelocity(deviation, initVel, timeOfFlight, initPos, "first", 100, .001);
            [maxErrorZVel2, maxError2] = self.findClosestVelocity(deviation, initVel, timeOfFlight, initPos, "last", 100, .001);
%             self.predictParabolic(0)
%             self.traj(end, :)

            minZVel = min([minErrorZVel, minErrorZVel2, maxErrorZVel, maxErrorZVel2]);
            maxZVel = max([minErrorZVel, minErrorZVel2, maxErrorZVel, maxErrorZVel2]);

            dataStart = minZVel;
            dataStop = maxZVel;
            dataStep = (maxZVel-minZVel)/100;
            wind0 = [0, 0, 0];
            zVels = dataStart:dataStep:dataStop;
            validZVels = [];
            deviations = [];
            allValid = true;
            for j = zVels
                initVel(3) = j;
                [~, ZVelError, actualTimeOfFlight, ~] = self.checkParameters(initVel, wind0, initPos, timeOfFlight);
                deviations(end+1, :) = [abs(ZVelError), j];
                [~, ~, newInitPredPosZ] = self.predictParabolic(actualTimeOfFlight);
                if ZVelError <= deviation && (newInitPredPosZ < initRange(2,1) && newInitPredPosZ > initRange(2,2))
                    validZVels(end + 1) = j;
                else
                    allValid = false;
                end
            end
%             plot(deviations(:, 2), deviations(:, 1))

            
            % Check if deviation is valid
            maxError = abs(initRange(2,1));
            minError = min([minError, minError2]);
            if deviation < minError
                ME = MException('TrajectoryCalculator:InvalidDeviation', ...
                    'Minimum Deviation: %f',minError);
                throw(ME)
            end
            if deviation > maxError
                ME = MException('TrajectoryCalculator:InvalidDeviation', ...
                    'Maximum Deviation: %f',maxError);
                throw(ME)
            end

            if isempty(validZVels)
                ME = MException('TrajectoryCalculator:NoSolutionFound', ...
                    'No solution found');
                throw(ME)
            end
            % Chose random z velocity
            if allValid
                zVelRange = maxZVel - minZVel;
                zVel = rand() * zVelRange + maxErrorZVel;
                
            else
                randIndex = ceil(rand() * length(validZVels));
                zVel= validZVels(randIndex);
            end
            initVel(3) = zVel;

            % Determine wind y velocity
            [newTimeOfFlightError, actualError, actualTimeOfFlight] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
            deviationError = deviation - actualError;
            while abs(deviationError) > tolerance
                [~, actualError, ~, actualEndPos] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
                deviationError = deviation - actualError;
                windVel(2) = windVel(2) * (1 + deviationError / deviation);
            end

            if rand() > .5
                windVel(2) = windVel(2) * -1;
            end

            self.calcTrajectory(initPos, initVel, windVel);
            [~, y0, z0] = self.predictParabolic(0);
            initPredPosWrong = [initPredPos(1), y0, z0];
            reqOffset = initPredPos - initPredPosWrong;
            initPos = initPos + reqOffset;

%             [~, actualError, actualTimeOfFlight] = self.checkParameters(initVel, windVel, initPos, timeOfFlight)
%             self.calcTrajectory(initPos, initVel, windVel);
%             self.traj(end, :)
%             [~, y0, z0] = self.predictParabolic(0)
        end

        % Generates 
        function [windVel, initVel, deviation, actualTimeOfFlight, initPos] = generateParameters3(self, initPredPos, distanceThrown, timeOfFlight, windVar, zVelVar, tolerance)
            % Initialize optional variables
            if ~exist('tolerance','var')
                tolerance = 0.01;
            end
            if ~exist('windVar','var')
                windVar = 2/timeOfFlight;
            end
            if ~exist('zVelVar','var')
                zVelVar = 5/timeOfFlight;
            end

            % Make initial velocity guesses
            xVelGuess = distanceThrown / timeOfFlight;
            yVelGuess = 0;
            zVelGuess = (rand() - .5) * 2 * zVelVar;

            % Assume initial position
            initPos = [-distanceThrown, 0, 0];

            % Guess wind velocity
            windXVelGuess = 0;
            windYVelGuess = (rand() - .5) * 2 * windVar; %Random +/-
%             windYVelGuess = rand() * windVar; % Random + only
            windZVelGuess = 0;

            % Initialize guess vectors
            initVel = [xVelGuess, yVelGuess, zVelGuess];
            windVel = [windXVelGuess, windYVelGuess, windZVelGuess];
            
            % Determine x velocity based on time of flight
            [newTimeOfFlightError, deviation, actualTimeOfFlight] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
            while abs(newTimeOfFlightError) > tolerance
                initVel(1) = initVel(1) * (1 + newTimeOfFlightError / actualTimeOfFlight);
                [newTimeOfFlightError, deviation, actualTimeOfFlight, actualEndPos] = self.checkParameters(initVel, windVel, initPos, timeOfFlight);
            end
            
            % Offset the initial position such that the first prediction
            % positiion lies at the desired location
            self.calcTrajectory(initPos, initVel, windVel);
            [~, y0, z0] = self.predictParabolic(0);
            initPredPosWrong = [initPredPos(1), y0, z0];
            reqOffset = initPredPos - initPredPosWrong;
            initPos = initPos + reqOffset;
            deviation = abs(deviation);
        end
    end
end