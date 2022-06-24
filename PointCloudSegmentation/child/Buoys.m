classdef Buoys < PointCloudSegmentation
    % WALLCORNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)

    end
    
    methods
        function this = Buoys()
            %WALLCORNER Construct an instance of this class
            %   Detailed explanation goes here
            this@PointCloudSegmentation();
        end
        
        function feature = SegementByAtribute(this, filteredPT)
            % Get first wall
            [modelCyl1, inlierIndices, outlierIndices] = pcfitcylinder(filteredPT, 0.02);
            cyl1 = select(filteredPT, inlierIndices);
            remainCloud = select(filteredPT, outlierIndices);

            [model1, inlierIndices1, outlierIndices1] = pcfitplane(remainCloud, 0.02, [1,0,0], 45);
            plane1 = select(remainCloud, inlierIndices1);
            remainCloud = select(remainCloud, outlierIndices1);
            
            [model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02, [1,0,0], 45);
            plane2 = select(remainCloud, inlierIndices2);

            % Extraire les point orienté
            [p1, q1] = this.getOrientedPointOnPlanarFace(model1, plane1);
            [p2, q2] = this.getOrientedPointOnPlanarFace(model2, plane2);

            if coder.target('MATLAB')
                figure('Name', 'Plane1');
                pcshow(plane1);
                hold on 
                pcshow(plane2);

                plot(model1);
                plot(model2);

                poseplot(quaternion(q1),'Position',p1);
                poseplot(quaternion(q2),'Position',p2);
            end
            feature = pcmerge(plane1, plane2, 0.01);
        end
    end
end
