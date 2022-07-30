classdef PointCloudSegmentation
    %POINTCLOULDSEGMENTATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        qUtils; % quaternion Utilities
        filteredPT; % filter point cloud

    end

    methods (Access = public )
        
        function this = PointCloudSegmentation(filterPT)
            this.qUtils = quatUtilities();
            this.filteredPT = filterPT;

        end
      
    end
    
    methods (Abstract, Access = public )
        feature = SegementByAtribute(this, pose);
    end

    methods(Access = protected)
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

        function  box = objectAllignBoudingBox(this,q, pc,model)

            tRot = quat2rotm(q);
            tf = rigid3d(tRot, [0 0 0]);
            pct = pctransform(pc, tf);
            
            box = zeros(1,3);
            box(1) = (pct.XLimits(2)-pct.XLimits(1));
            box(2) = (pct.YLimits(2)-pct.YLimits(1));
            box(3) = (pct.ZLimits(2)-pct.ZLimits(1));
            
        
        end

        function qFlip = isObstaclePoseFlipped(this, obsQuat, auvQuat)
            qFlip = obsQuat;
            angle = abs(this.qUtils.angleBetween2Quaternion(obsQuat, auvQuat));
            if min(2*pi - angle, angle) > pi / 2
                qFlip = quatmultiply(obsQuat, eul2quat([pi 0 0], "ZYX"));
            end
        end
    end
end

