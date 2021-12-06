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
        firstConfig
        model1
        model2
        anf
        configSol_anf
    end

    methods
        function self = BallCatcher()
            % Default DH parameters
            if ~exist('dhparams','var')
                dhparams = [0         pi/2	  0 	pi/2;
                            0.9144	  0       0 	0;  % Basic 2d planar robot dh table
                            0.9144	  0       0 	0;
                            0         0       0 	0];
            end
            self.dhparams = dhparams;
            self.q_dot_1 = 2; %rad/s
            self.q_dot_2 = 2; %rad/s
            %self.weights = [0.0025 0.0025 0.0025 0.01 0.01 0.01];
            self.weights = [0 0 0 0 .8 .8];

            % Create robot arm
            self.setupArm();
            % Set initial config
            self.firstConfig = homeConfiguration(self.robot);
            self.firstConfig(1).JointPosition = 5*pi/8;
            self.firstConfig(2).JointPosition = -5*pi/8;
            self.lastConfig = self.firstConfig;

            %setting up anfis
            self.model1 = readfis('theta1_full_model_testing6.fis');
            self.model2 = readfis('theta2_full_model_testing6.fis');
            %model3 = readfis('theta3_full_model');

            self.anf = ANFIS(5,150,0); 
            field = 'JointPosition';
            field2 = 'JointName';
            self.configSol_anf = struct(field,{},field2,{});                
            self.configSol_anf(1,1).JointName = "jnt2";
            self.configSol_anf(1,2).JointName = "jnt3";
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
            %body4 = robotics.RigidBody('body4');
            %jnt4 = robotics.Joint('jnt4','revolute');
            body4 = robotics.RigidBody('tool');
            jnt4 = robotics.Joint('jnt4','fixed');
            
            % addVisual(body2,"Mesh",'link_body.stl')
            
%             addCollision(body3,collisionCylinder(0.127,0.9144));

            %Calculate transformation matrices for each joint
            setFixedTransform(jnt1,self.dhparams(1,:),'dh');
            setFixedTransform(jnt1,self.dhparams(1,:),'dh');
            setFixedTransform(jnt2,self.dhparams(2,:),'dh');
            setFixedTransform(jnt3,self.dhparams(3,:),'dh');
            setFixedTransform(jnt4,self.dhparams(4,:),'dh');
            %setFixedTransform(jnt5,self.dhparams(5,:),'dh');
            
            %Assign joints to each link body
            body1.Joint = jnt1;
            body2.Joint = jnt2;
            body3.Joint = jnt3;
            body4.Joint = jnt4;
           %body5.Joint = jnt5;
            
            %Set limits of each joint
            jnt2.PositionLimits = [-pi pi];
            jnt3.PositionLimits = [-pi pi];
            %jnt4.PositionLimits = [-pi pi];
        
            %Create each link's body within the robot structure
            addBody(self.robot,body1,'base')
            addBody(self.robot,body2,'body1')
            addBody(self.robot,body3,'body2')
            addBody(self.robot,body4,'body3')
            %addBody(self.robot,body5,'body4')

            % Initialize inverse kinematics
            self.ik = inverseKinematics('RigidBodyTree',self.robot);
        end

        % Use inverse kinematics or ANFIS to determine joint angles
        function [configSol, solTime] = calcRobotPos(self, algorithm, targetPos)
            % Inverse Kinematics
            if algorithm == "ik"
                tic;
                [configSol,~] = self.ik('tool',trvec2tform(targetPos),self.weights,self.lastConfig);
%                 targetPos
%                 trvec2tform(targetPos)
%                 self.getEndEffectorPos(configSol)
                solTime = toc;
            else
                tic;
                anfis_jntangle_1 = evaluate(self.anf,self.model1,targetPos(:,2:3));
                anfis_jntangle_2 = evaluate(self.anf,self.model2,targetPos(:,2:3)); % implement ANFIS here
                solTime = toc;
                field = 'JointPosition';
                field2 = 'JointName';
                configSol = struct(field,{},field2,{});     
                configSol(1,1).JointPosition = anfis_jntangle_1;
                configSol(1,2).JointPosition = anfis_jntangle_2;
                configSol(1,1).JointName = "jnt2";
                configSol(1,2).JointName = "jnt3";
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
            xlim([-20 3]);
            ylim([-2 2]);
            zlim([-2.5 4]);
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

