classdef ANFIS
    properties
        mem_fxn;
        epochs;
        display_info;
    end

    methods
        function self = ANFIS(num_member_fxn,num_epoch,display)
            self.mem_fxn = num_member_fxn;
            self.epochs = num_epoch;
            self.display_info = display;
        end

        function model = train(self,data)
        
            opt = anfisOptions;
            opt.InitialFIS = self.mem_fxn;
            opt.EpochNumber = self.epochs;
            opt.DisplayANFISInformation = self.display_info;
            opt.DisplayErrorValues = self.display_info;
            opt.DisplayStepSize = self.display_info;
            opt.DisplayFinalResults = self.display_info;
          
            model = anfis(data,opt);
        end

        function output_xy = evaluate(self,trained_model,input_xy)
            output_xy = evalfis(trained_model,input_xy);
        end

        function [error_theta1 ,error_theta2] = error(self,predicted_theta1,actual_theta1,predicted_theta2,actual_theta2)
            disp("Actual : " + size(actual_theta1)+ "  "+ size(actual_theta2));
            disp("Predicted : " + size(predicted_theta1)+ "  "+ size(predicted_theta2));
            error_theta1 = actual_theta1(:)-predicted_theta1(:);
            error_theta2 = actual_theta2(:)-predicted_theta2(:);
        end
    end
end