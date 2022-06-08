classdef WallCorner < PointClouldSegmentation
    %WALLCORNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property
    end
    
    methods
        function obj = WallCorner()
            %WALLCORNER Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function feature = SegementByAtribute(this,filteredPT);
            % Get first wall
            [model1, inlierIndices, outlierIndices] = pcfitplane(denoiseCloud, 0.02);
            plane1 = select(denoiseCloud, inlierIndices);
            remadenoisinCloud = select(denoiseCloud, outlierIndices);

            % Get second wall
            [model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02);
            plane2 = select(remainCloud, inlierIndices2);

            if coder.target('MATLAB')
                pcshow(plane1);
                hold on 
                pcshow(plane2);

                plot(model1);
                plot(model2);
            end
            
        end
    end
end

