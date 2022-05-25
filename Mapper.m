classdef Mapper < handle
    % MAPPER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % map3D; % Occupancy map.
        bigCloud;
        player;
    end
    
    methods
        %% Mapper Constructor
        function this = Mapper()           
            % this.map3D = occupancyMap3D(10);
            this.bigCloud = pointCloud(zeros([1, 3]));
            this.player = pcplayer([0 30],[-30 30],[0 5]);
        end

        %% Sonar laser scan callback.
        function add2PtCloud(this, sonarMsg, poseSub)
            fprintf('INFO : proc mapping : Append to point cloud. \n');
            % scan = rosReadLidarScan(sonarMsg);
            pos = [0, 0, 0];
            quat = [1, 0, 0, 0];
            
            % Getting the sub pose.
            if ~isempty(poseSub.LatestMessage)
               fprintf("INFO : proc mapping : Pose received. \n");
               pos(1) = poseSub.LatestMessage.Position.X;
               pos(2) = poseSub.LatestMessage.Position.Y;
               pos(3) = poseSub.LatestMessage.Position.Z;              

               quat(1) = poseSub.LatestMessage.Orientation.W;
               % Quaternion conjugate.
               quat(2) = -poseSub.LatestMessage.Orientation.X;
               quat(3) = -poseSub.LatestMessage.Orientation.Y;
               quat(4) = -poseSub.LatestMessage.Orientation.Z;
            end
            cartesians = rosReadXYZ(sonarMsg);
            intensities = rosReadField(sonarMsg, 'intensity');

            xyzPoints = zeros([size(cartesians, 1), 3]);
            for i=1:size(cartesians, 1)
                point = sonar2NED(pos.', quat, [0.358, 0, -0.118].', [cartesians(i,1), cartesians(i,2), 0]).'; 
                xyzPoints(i, :) = point(1:3);
            end
            ptCloud = pointCloud(xyzPoints, 'Intensity', intensities);   
            this.bigCloud = pccat([this.bigCloud, ptCloud]);
            view(this.player, this.bigCloud);
            % this.bigCloud = pcmerge(this.bigCloud, ptCloud, 1);
            % insertPointCloud(this.map3D, [0,0,0,1,0,0,0], ptCloud, double(sonarMsg.RangeMax));
        end
    end
end
