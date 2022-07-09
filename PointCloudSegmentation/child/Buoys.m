classdef Buoys < PointCloudSegmentation
    % WALLCORNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)        
        PTlabels;
        buoyPT;

        % ROS Parameters.
        param;
    end
    %==============================================================================================
    % Public functions
    %==============================================================================================   
    methods
        function this = Buoys(filterPT, param)
            %WALLCORNER Construct an instance of this class
            %   Detailed explanation goes here
            this@PointCloudSegmentation(filterPT);
            
            this.param = param;
            
            this.PTlabels = uint32(zeros(1,this.filteredPT.Count));

            if coder.target('MATLAB')
                this.buoyPT  = pcdownsample(pcread('buoy.ply'),'gridAverage',0.01);    
                pcshow(this.filteredPT);
                hold on
            else
                buoysFile = coder.load('MAT/buoyXYZ.mat');
                this.buoyPT = pointCloud(buoysFile.buoys);
                
            end

            this.buoyPT.Intensity = ones(this.buoyPT.Count,1)*0.1;
            this.buoyPT.Normal = zeros(this.buoyPT.Count,3);

        end
        
        function feature = SegementByAtribute(this)
            
            % Get clusters
            [this.PTlabels,numClusters] = pcsegdist(this.filteredPT,this.param.clusterDist);
            goodCluster = zeros(1, numClusters);

            for i =1  : numClusters

                goodCluster(i) = this.analyseCluster(i);

            end

            switch sum(goodCluster)

                % Suspect 2 buyos in the same clusters
                case 1
                    % get the good cluster
                    index = find(goodCluster == 1);
                    clusterLabels = this.PTlabels == index;
                    clusterPT =  select(this.filteredPT, clusterLabels);
                    
                    % split cluster with kmeans
                    kmeansIndex = kmeans(clusterPT.Location, 2);
                                    
                    %for each buoys
                    for i = 1 : 2 
                        
                        [p,q] = this.getBuoyPose(select(clusterPT, kmeansIndex == i));
            
                    end

                otherwise
            end
            feature = true; 
        end
    end

    %==============================================================================================
    % Privates functions
    %==============================================================================================
    methods(Access = private)

        function isPotential = analyseCluster(this,i)

            % extract pointCloud
            clusterLabels = this.PTlabels == i;
            clusterPT = select(this.filteredPT,clusterLabels);
    
            % fit plane on cluster
            [model, indexOnPlane, ~ , meanError ] = pcfitplane(clusterPT, this.param.planeTol );
    
            plane =  select(clusterPT, indexOnPlane);
    
            % Get Z normal
            zNormal = model.Normal(3);
    
            % Ratio in plane
            percentInPlane = max(size(indexOnPlane)) / clusterPT.Count;
    
            % Extract pose of the plane.
            [p, q] = this.getOrientedPointOnPlanarFace(model, plane);
    
            % extract bounding box
            box = this.objectAllignBoudingBox(q, plane,model);

            % find area
            area = box(2)*box(3);
    
            % Check if cluster is a potential buoys
            if (zNormal < this.param.zNormalThres...
                && percentInPlane > this.param.inPlaneThres...
                && area > this.param.minArea && area < this.param.maxArea)
        
                isPotential = 1;
            else

                isPotential = 0;
            end

        end

        function [p,q] = getBuoyPose(this, subPT)
            
            % Apply ransac 
            [model, indexOnPlane, ~, meanError ] = pcfitplane(subPT, this.param.planeTol );
            plane =  select(subPT, indexOnPlane);
            
            % Get ransac plane pose approximation 
            [pApprox,qApprox] = this.getOrientedPointOnPlanarFace(model, subPT);
            
            % Transform the buoy on the plane.
            tformRansac = rigid3d(quat2rotm(quatinv(qApprox)), pApprox);
            buoyTformed = pctransform(this.buoyPT, tformRansac );
                       
            % Apply icp.
            tformICP = pcregistericp(buoyTformed , plane,"InlierRatio",this.param.icpInlierRatio);

            % Get buoys transformation.
            tformBuoy = rigid3d(tformRansac.T * tformICP.T);
            
            % return transform
            p = tformBuoy.Translation;
            q = rotm2quat(tformBuoy .Rotation.');

            if coder.target('MATLAB')
                pcshow(buoyTformed)
                hold on 
                poseplot(quaternion(rotm2quat(tformBuoy.Rotation.')),'Position',tformBuoy.Translation,ScaleFactor=0.2);
            end
        

        end
    end
end
