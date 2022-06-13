classdef WallCorner < PointClouldSegmentation
    %WALLCORNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    
    methods
        function obj = WallCorner()
            %WALLCORNER Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function feature = SegementByAtribute(this, filteredPT)
            % Get first wall
            [model1, inlierIndices, outlierIndices] = pcfitplane(filteredPT, 0.02);

            plane1 = select(filteredPT, inlierIndices);
            remainCloud = select(filteredPT, outlierIndices);
            
            [model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02);
            plane2 = select(remainCloud, inlierIndices2);

            if coder.target('MATLAB')
                figure('Name', 'Plane1');
                pcshow(plane1);
                hold on 
                pcshow(plane2);

                plot(model1);
                plot(model2);
            end
            feature = pcmerge(plane1, plane2, 0.01);
        end
    end
end

