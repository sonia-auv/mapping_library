classdef PointCloudSegmentation
    %POINTCLOULDSEGMENTATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        qUtils;
    end

    methods (Access = public )
        
        function this = PointCloudSegmentation()
            this.qUtils = quatUtilities();
        end
      
    end
    
    methods (Abstract, Access = public )
        
        feature = SegementByAtribute(this,filteredPT);
      
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
    end
end

