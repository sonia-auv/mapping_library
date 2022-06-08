classdef RosNode
    %ROSNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        
    end
    
    methods
        function obj = RosNode()
            
        end
        
        %% ROS Spin
        function spin(this, spin)
            killNode = false;
            reset(spin);
            fprintf('INFO : proc mapping : Node is started. \n');
            fprintf('INFO : proc mapping : Wait for point cloud. \n');

            bundler = PointCloudBundler();

            while ~killNode
                if ~bundler.step()
                    fprintf('INFO : proc mapping : Not bundling. \n');
                    ptCloud = pointCloud(bundler.getBundle(':', 1:3), 'Intensity', bundler.getBundle(':', 4));  
                else
                    % fprintf('INFO : proc mapping : Bundling or waiting. \n');
                end
                waitfor(spin);
            end
        end
    end
end

