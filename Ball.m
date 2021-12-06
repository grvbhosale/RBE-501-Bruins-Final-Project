classdef Ball < handle
    properties
        mass
        radius
        dragCoefficient
        crossSectionalArea
        terminalVelocity
    end
    methods
        function self = Ball(mass, radius, drag)
            if ~exist('drag','var')
                drag = .5; %kg / m^3;
            end
            self.mass = mass; % kg
            self.radius = radius; % m
            self.dragCoefficient = drag;
            self.crossSectionalArea = 1/2 * pi * radius; % m^2
%             self.calcTerminalVelocity()
        end
        
        function airResistance = calculateDrag(self, ballVel, windVel, airDensity)
            if ~exist('airDensity','var')
                airDensity = 1.225; %kg / m^3;
            end

            relativeVelocity = windVel - ballVel;
            airResistance = 1/2 * airDensity * relativeVelocity.^2 * self.dragCoefficient * 1/2 * self.crossSectionalArea;
            airResistance = airResistance .* relativeVelocity ./ abs(relativeVelocity);
            if relativeVelocity(1) == 0
                airResistance(1) = 0;
            end
            if relativeVelocity(2) == 0
                airResistance(2) = 0;
            end
            if relativeVelocity(3) == 0
                airResistance(3) = 0;
            end
        end

        function calcTerminalVelocity(self)
            if ~exist('airDensity','var')
                airDensity = 1.225; %kg / m^3;
            end
            syms velSym
            Faz = 1/2 * airDensity * velSym * self.dragCoefficient * 1/2 * self.crossSectionalArea;
            Fgz = -self.mass * 9.81;
            self.terminalVelocity = double(vpa(solve(Faz == Fgz)));
        end
    end
end