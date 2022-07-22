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
        
        function feature = SegementByAtribute(this, auvQuat)            
            % Get clusters
            [this.PTlabels,numClusters] = pcsegdist(this.filteredPT,this.param.clusterDist);
            goodCluster = zeros(1, numClusters);

            obstacle = rosmessage("sonia_common/ObstacleInfo", "DataFormat", "struct");
            feature = repelem(obstacle, 2);

            for i =1  : numClusters
                goodCluster(i) = this.analyseCluster(i);
            end

            switch sum(goodCluster)
                % Suspect 2 buyos in the same clusters
                case 1
                    index = find(goodCluster == 1);
                    % get the good cluster
                    clusterLabels = this.PTlabels == index;
                    clusterPT =  select(this.filteredPT, clusterLabels);
                    
                    % split cluster with kmeans
                    kmeansIndex = kmeans(clusterPT.Location, 2);

                    % for each buoys
                    for i = 1 : 2
                        [p, q, confidence] = this.getBuoyPose(select(clusterPT, kmeansIndex == i), auvQuat)
                        obstacle.IsValid = true;
                        obstacle.Name = char('Buoys');
                        obstacle.Confidence = single(confidence);
                        obstacle.Pose.Position.X = p(1);
                        obstacle.Pose.Position.Y = p(2);
                        obstacle.Pose.Position.Z = p(3);
                        obstacle.Pose.Orientation.W = q(1);
                        obstacle.Pose.Orientation.X = q(2);
                        obstacle.Pose.Orientation.Y = q(3);
                        obstacle.Pose.Orientation.Z = q(4);
                        feature(i) = obstacle;
                    end

                case 2
                    index = find(goodCluster == 1);
                    % for each buoys
                    for i = 1 : 2
                        clusterLabels = this.PTlabels == index(i);
                        [p, q, confidence] = this.getBuoyPose(select(this.filteredPT, clusterLabels), auvQuat) 
                        obstacle.IsValid = true;
                        obstacle.Name = char('Buoys');
                        obstacle.Confidence = single(confidence);
                        obstacle.Pose.Position.X = p(1);
                        obstacle.Pose.Position.Y = p(2);
                        obstacle.Pose.Position.Z = p(3);
                        obstacle.Pose.Orientation.W = q(1);
                        obstacle.Pose.Orientation.X = q(2);
                        obstacle.Pose.Orientation.Y = q(3);
                        obstacle.Pose.Orientation.Z = q(4);
                        feature(i) = obstacle;
                    end

                % Nothing found
                otherwise
                    for i = 1 : 2
                        feature(i).IsValid = false;

                    end
                    return
            end 
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

            % Test fit cylinder
%             [modelCyl, indexOnCyl, outlierIndexCyl] = pcfitcylinder(clusterPT, 0.01 );
%             rest =  select(clusterPT, outlierIndexCyl);
    
            % fit plane on cluster
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
    
            % extract bounding box
            box = this.objectAllignBoudingBox(q, plane,model);

            % find area
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
            
            % Apply ransac 
            [model, indexOnPlane, ~, meanError ] = pcfitplane(subPT, this.param.planeTol, [1,0,0], 45 );
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
            
            % Verify plane confidence
            confidence = 100;

            % Get Z normal
            zNormal = model.Normal(3);
            confidence = confidence * (1- abs(zNormal));

            % Ratio in plane
            percentInPlane = max(size(indexOnPlane)) / subPT.Count;
            confidence = confidence * percentInPlane;
    


            % return transform and confidence
            p = tformBuoy.Translation;
            q = this.isObstaclePoseFlipped(rotm2quat(tformBuoy.Rotation.'), auvQuat);

            % extract bounding box
            box = this.objectAllignBoudingBox(q, plane,model);
            % find area
            area = box(2)*box(3);
            if area < this.param.minArea || area > this.param.maxArea
                confidence = confidence / 2;
            end

            if coder.target('MATLAB')
                pcshow(buoyTformed)
                hold on 
                poseplot(quaternion(q),'Position',p,ScaleFactor=0.2);
            end
        

        end
    end
end
