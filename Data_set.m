classdef Data_set < handle

    properties
        TimeOfFlight
        InitialPosition
        InitialVelocity
        Deviation
        Data_raw
        Mean_ik
        Mean_anfis
        Interval_ik
        Interval_anfis
    end
    
    methods
        function self = Data_set(timeOfFlight)
            self.TimeOfFlight = timeOfFlight;
            self.InitialPosition = [];
            self.InitialVelocity = [];
            self.Deviation = [];
            self.Data_raw = [];
        end
    end
end
