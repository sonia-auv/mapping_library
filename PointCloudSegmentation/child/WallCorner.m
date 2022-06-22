classdef WallCorner < PointClouldSegmentation
    % WALLCORNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        qUtils; % quaternion utilities
    end
    
    methods
        function this = WallCorner()
            %WALLCORNER Construct an instance of this class
            %   Detailed explanation goes here
            this.qUtils = quatUtilities();
        end
        
        function feature = SegementByAtribute(this, filteredPT)
            % Get first wall
            [model1, inlierIndices, outlierIndices] = pcfitplane(filteredPT, 0.02);

            plane1 = select(filteredPT, inlierIndices);
            remainCloud = select(filteredPT, outlierIndices);
            
            % [model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02);
            % plane2 = select(remainCloud, inlierIndices2);

            % Extraire les point orienté
            [p1, q1] = this.getOrientedPointOnPlanarFace(model1, plane1);
            % [p2, q2] = this.getOrientedPointOnPlanarFace(model2, plane2);

            if coder.target('MATLAB')
                figure('Name', 'Plane1');
                pcshow(plane1);
                hold on 
                % pcshow(plane2);

                plot(model1);
                % plot(model2);

                poseplot(quaternion(q1),'Position',p1);
                % poseplot(quaternion(q2),'Position',p2);
            end
            feature = [q1, p1];
            %feature = pcmerge(plane1, plane2, 0.01);
        end
    end

    methods(Access = private)
        function [p,q] = getOrientedPointOnPlanarFace(this,model, plane)

            % vecteur initial
            v1 =[1,0,0];
            
            % Trouver la transformaion angulaire du plan
            q = this.qUtils.quaternionForm2Vectors(v1, model.Normal);
            
            % Trouver la transformation linéaire du plan
            p = zeros(1,3);
            p(1) = (plane.XLimits(2)-plane.XLimits(1))/2 + plane.XLimits(1);
            p(2) = (plane.YLimits(2)-plane.YLimits(1))/2 + plane.YLimits(1);
            p(3) = (plane.ZLimits(2)-plane.ZLimits(1))/2 + plane.ZLimits(1);
        end
    end
end

