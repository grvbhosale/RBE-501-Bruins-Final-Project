% Robot designed to catch a ball, 2 DOF
classdef BallCatcher < handle
    properties
        dhparams
        robot
        q_dot_1
        q_dot_2
        ik
        weights
        lastConfig
    end

    methods
        function self = BallCatcher(dhparams)
            % Default DH parameters
            if ~exist('dhparams','var')
                dhparams = [0         pi/2	  0 	pi/2;
                            09.144	  0       0 	0;  % Basic 2d planar robot dh table
                            09.144	  0       0 	0;
                            0         0       0 	0];
            end
            self.dhparams = dhparams;
            self.q_dot_1 = 1.5; %rad/s
            self.q_dot_2 = 1.5; %rad/s
            self.weights = [0.0025 0.0025 0.0025 0.01 0.01 0.01];

            % Create robot arm
            self.setupArm();
            % Set initial config
            self.lastConfig = homeConfiguration(self.robot);
            
        end
        
        function setupArm(self)
            self.robot = robotics.RigidBodyTree; % Robot structure within RST
        
            %Assign names to bodies and joints and determine type
            body1 = robotics.RigidBody('body1');
            jnt1 = robotics.Joint('jnt1','fixed');
            body2 = robotics.RigidBody('body2');
            jnt2 = robotics.Joint('jnt2','revolute');
            body3 = robotics.RigidBody('body3');
            jnt3 = robotics.Joint('jnt3','revolute');
            body4 = robotics.RigidBody('tool');
            jnt4 = robotics.Joint('jnt4','fixed');
            
            % addVisual(body2,"Mesh",'link_body.stl')
            
            addCollision(body3,collisionCylinder(0.127,0.9144));

            %Calculate transformation matrices for each joint
            setFixedTransform(jnt1,self.dhparams(1,:),'dh');
            setFixedTransform(jnt1,self.dhparams(1,:),'dh');
            setFixedTransform(jnt2,self.dhparams(2,:),'dh');
            setFixedTransform(jnt3,self.dhparams(3,:),'dh');
            setFixedTransform(jnt4,self.dhparams(4,:),'dh');
            
            %Assign joints to each link body
            body1.Joint = jnt1;
            body2.Joint = jnt2;
            body3.Joint = jnt3;
            body4.Joint = jnt4;
            
            %Set limits of each joint
            jnt2.PositionLimits = [-pi pi];
            jnt3.PositionLimits = [-pi pi];
        
            %Create each link's body within the robot structure
            addBody(self.robot,body1,'base')
            addBody(self.robot,body2,'body1')
            addBody(self.robot,body3,'body2')
            addBody(self.robot,body4,'body3')

            % Initialize inverse kinematics
            self.ik = inverseKinematics('RigidBodyTree',self.robot);
        end

        % Use inverse kinematics or ANFIS to determine joint angles
        function [configSol, solTime] = calcRobotPos(self, algorithm, targetPos)
            % Inverse Kinematics
            if algorithm == "ik"
                tic;
                [configSol,~] = self.ik('tool',trvec2tform(targetPos),self.weights,self.lastConfig);
                solTime = toc;
            % ANFIS (TBD)
            else
                tic;
                configSol = homeConfiguration(self.robot); % implement ANFIS here
                solTime = toc;
            end
        end

        % Display robot on graph with ball and predicted ball
        function showRobot(self, config, predictedPos, actualPos)
            % Display predicted ball
            scatter3(predictedPos(1), predictedPos(2), predictedPos(3),'o','MarkerEdgeColor','r');
            hold on
            % Show Robot
            show(self.robot,config,'Visuals','on');
            % Display actual ball
            scatter3(actualPos(1), actualPos(2), actualPos(3),'o','MarkerEdgeColor','r');
            % Set axis limits
            xlim([-20 20]);
            ylim([-20 20]);
            zlim([-20 20]);
            hold off
            % Pause to update graphics
            pause(.01);
        end

        % Move arm in direction of target position
        function newConfig = moveArm(self, targetConfig, timeInterval) %Currently no torque or accelerataion control
            % Should generate trajectory based on torque and speed
            % limitations and set end position based on position at time
            % interval rather than assuming infinte torque and constant
            % velocity along the travel of each movement

            % Initialize new config
            newConfig = self.lastConfig;

            % Store and joint positions in an easier-to-read variable
            TJA1 = BallCatcher.normalizeAngle(targetConfig(1,1).JointPosition);
            TJA2 = BallCatcher.normalizeAngle(targetConfig(1,2).JointPosition);
            LJA1 = BallCatcher.normalizeAngle(self.lastConfig(1,1).JointPosition);
            LJA2 = BallCatcher.normalizeAngle(self.lastConfig(1,2).JointPosition);

            % Calculate joint travel desired
            delta_j1 = BallCatcher.normalizeAngle(TJA1 - LJA1);
            delta_j2 = BallCatcher.normalizeAngle(TJA2 - LJA2);

            % Determine max joint travel permissible
            maxDeltaJ1 = timeInterval * self.q_dot_1;
            maxDeltaJ2 = timeInterval * self.q_dot_2;

            % If joint wants to travel faster than is allowed, set joint
            % change to maximum permissible change
            if abs(delta_j1) > maxDeltaJ1
                delta_j1 = BallCatcher.normalizeAngle(maxDeltaJ1 * sign(delta_j1));
            end
            if abs(delta_j2) > maxDeltaJ2
                delta_j2 = BallCatcher.normalizeAngle(maxDeltaJ2 * sign(delta_j2));
            end                

            % Calculate new position by adding angle change to last position
            newJ1 = BallCatcher.normalizeAngle(LJA1 + delta_j1);
            newJ2 = BallCatcher.normalizeAngle(LJA2 + delta_j2);

            % Update configuration
            newConfig(1, 1).JointPosition = newJ1;
            newConfig(1, 2).JointPosition = newJ2;
            self.lastConfig = newConfig;
        end

        % Get x, y, z coordinates of end effector
        function pos = getEndEffectorPos(self, config)
            transform = getTransform(self.robot,config,'tool','base');
            pos = transform(1:3, 4)';
        end
    end
    methods(Static)
        % Normalize angle to be from -pi to pi
        function angle = normalizeAngle(angle)
            while angle < -pi
                angle = angle + 2 * pi;
            end
            while angle > pi
                angle = angle - 2 * pi;
            end
        end
    end
end

