classdef Buoys < PointCloudSegmentation
    % BUOYS Summary of this class goes here
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
            % BUOYS Construct an instance of this class
            %   Detailed explanation goes here
            this@PointCloudSegmentation(filterPT);
            
            this.param = param;
            
            this.PTlabels = uint32(zeros(1,this.filteredPT.Count));

            if coder.target('MATLAB')
                % generate obstacle reference PT
                buoy = pcdownsample(pcread('buoy.ply'),'gridAverage',0.01);
            else
                buoysFile = coder.load('MAT/buoyXYZ.mat');
                buoy = pointCloud(buoysFile.buoys);        
            end

            % apply offset gaps on  pannel 
            T1 = rigid3d(quat2rotm([1,0,0,0]), [0, this.param.gap / 2, 0]);
            T2 = rigid3d(quat2rotm([1,0,0,0]), [0, -this.param.gap / 2, 0]);

            buoy1 = pctransform(buoy, T1);
            buoy2 = pctransform(buoy, T2);
            this.buoyPT  = pccat([buoy1, buoy2]);
            
            this.buoyPT.Intensity = ones(this.buoyPT.Count,1)*0.1;
            this.buoyPT.Normal = zeros(this.buoyPT.Count,3);
        end
        
        function feature = SegementByAtribute(this, auvPose)            
            % Get clusters
            [this.PTlabels,numClusters] = pcsegdist(this.filteredPT,this.param.clusterDist);
            goodCluster = zeros(1, numClusters);

            pcshow(this.filteredPT.Location, this.PTlabels);
            colormap(hsv(numClusters));

            obstacle = rosmessage("sonia_common/ObstacleInfo", "DataFormat", "struct");
            feature = repelem(obstacle, 2);

            % Get the good clusters.
            for i = 1 : numClusters
                goodCluster(i) = this.analyseCluster(i);
            end
            
            % One or more clusters found.
            if sum(goodCluster)
                fprintf("INFO : proc mapping : sonar :  %d clusters found. \n", sum(goodCluster));
                distances = zeros(1, sum(goodCluster));
                for i = 1 : sum(goodCluster)
                    index = find(goodCluster == 1);
                    clusterLabels = this.PTlabels == index(i);
                    clusterPT = select(this.filteredPT, clusterLabels);
                    
                    [p, q, confidence] = this.getBuoyPose(clusterPT, auvPose(4:7))
                    distances(i) = pdist([p(1:3) ; auvPose(1:3)]);
                end
                [~, closestClusterId] = min(distances);
                goodCluster = zeros(1, numClusters);
                goodCluster(closestClusterId) = 1;
    
                % Keep the closest cluster.
                index = find(goodCluster == 1);
                clusterLabels = this.PTlabels == index;
                clusterPT = select(this.filteredPT, clusterLabels);

                [p, q, confidence] = this.getBuoyPose(clusterPT, auvPose(4:7));
                obstacle.IsValid = true;
                obstacle.Name = char('Buoys');
                obstacle.Confidence = single(confidence);
    
                offset = this.qUtils.quatRotation([0,this.param.gap / 2, 0], q);
    
                disp('Panel #1');
                obstacle.Pose.Position.X = p(1) + offset(1);
                obstacle.Pose.Position.Y = p(2) + offset(2);
                obstacle.Pose.Position.Z = p(3) + offset(3);
                obstacle.Pose.Orientation.W = q(1);
                obstacle.Pose.Orientation.X = q(2);
                obstacle.Pose.Orientation.Y = q(3);
                obstacle.Pose.Orientation.Z = q(4);
                
                feature(1) = obstacle;
                disp('Panel #2');
                obstacle.Pose.Position.X = p(1) - offset(1);
                obstacle.Pose.Position.Y = p(2) - offset(2);
                obstacle.Pose.Position.Z = p(3) - offset(3);
                feature(2) = obstacle;
    
                if coder.target('MATLAB')
                    hold on 
                    poseplot(quaternion(q),'Position',p,ScaleFactor=0.2);
                    poseplot(quaternion(q),'Position',p + offset,ScaleFactor=0.2);
                    poseplot(quaternion(q),'Position',p - offset,ScaleFactor=0.2);
                end
            % No clusters found.
            else
                fprintf("INFO : proc mapping : sonar : No clusters found. \n");
                obstacle.IsValid = false;
                obstacle.Name = char('Buoys');
                obstacle.Confidence = single(0);
                feature(1) = obstacle;

                feature(2) = obstacle;
            end
        end
    end

    %==============================================================================================
    % Privates functions
    %==============================================================================================
    methods(Access = private)

        function isPotential = analyseCluster(this,i)

            % Extract pointCloud
            clusterLabels = this.PTlabels == i;
            clusterPT = select(this.filteredPT,clusterLabels);
    
            % Fit plane on cluster
            [model, indexOnPlane, ~ , meanError ] = pcfitplane(clusterPT, this.param.planeTol, [1,0,0],45 );
            plane =  select(clusterPT, indexOnPlane);

            if isempty(plane.Location)
                isPotential = 0;
                return
            end
    
            % Get Z normal
            zNormal = model.Normal(3);
    
            % Ratio in plane
            percentInPlane = max(size(indexOnPlane)) / clusterPT.Count;
    
            % Extract pose of the plane.
            [p, q] = this.getOrientedPointOnPlanarFace(model, plane);
    
            % Extract bounding box
            box = this.objectAllignBoudingBox(q, plane,model);

            % Find area
            area = box(2)*box(3);

            % Check if cluster is a potential buoys
            if (abs(zNormal) < this.param.zNormalThres...
                && percentInPlane > this.param.inPlaneThres...
                && area > this.param.minArea && area < this.param.maxArea)
        
                isPotential = 1;
                if coder.target('MATLAB')
                    figure(i+4);
                    pcshow(plane);
                    hold on;
                    model.plot;
                end
            else

                isPotential = 0;
            end

        end

        function [p,q,confidence] = getBuoyPose(this, subPT, auvQuat)
            
            % Apply Ransac 
            [model, indexOnPlane, ~, meanError ] = pcfitplane(subPT, this.param.planeTol, [1,0,0], 45 );
            plane =  select(subPT, indexOnPlane);
            
            % Get ransac plane pose approximation 
            [pApprox,qApprox] = this.getOrientedPointOnPlanarFace(model, subPT);
            
            % Transform the buoy on the plane.
            tformRansac = rigid3d(quat2rotm(qApprox), pApprox);
            buoyTformed = pctransform(this.buoyPT, tformRansac );
                       
            % Apply icp.
            tformICP = pcregistericp(buoyTformed , plane,"InlierRatio",this.param.icpInlierRatio);

            % Get buoys transformation.
            tformBuoy = rigid3d(tformRansac.T * tformICP.T);
            
            % Verify plane confidence
            confidence = 100;

            % Get Z normal
            zNormal = model.Normal(3);
            confidence = confidence * (1 - abs(zNormal));

            % Ratio in plane
            percentInPlane = max(size(indexOnPlane)) / subPT.Count;
            confidence = confidence * percentInPlane;
    
            % Teturn transformation and confidence
            p = tformBuoy.Translation;
            q = this.isObstaclePoseFlipped(rotm2quat(tformBuoy.Rotation.'), auvQuat);

            % Extract bounding box
            box = this.objectAllignBoudingBox(q, plane,model);
            % Find area
            area = box(2)*box(3);
            if area < this.param.minArea || area > this.param.maxArea
                confidence = confidence / 2;
            end

            %pcshow(buoyTformed)
           
        end
    end
end
