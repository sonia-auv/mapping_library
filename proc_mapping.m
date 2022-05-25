function proc_mapping
    % Global variables
    global newSonarMsg newSonarPtMsg;
    newSonarMsg = false;
    newSonarPtMsg = false;
    
    % Variables
    rosSpin = 10;
    r = rosrate(rosSpin);
    killNode = false;     
 
    % Subscribers
    poseSub = rossubscriber('/proc_nav/auv_pose', 'geometry_msgs/Pose', "DataFormat", "struct");
    sonarSub = rossubscriber('/provider_sonar/point_cloud2', 'sensor_msgs/PointCloud2', @sonarCallback, "DataFormat", "struct");
    
    % Objects
    mapper = Mapper();
    
    fprintf('INFO : proc mapping : Node is started \n');
    fprintf('INFO : proc mapping : Wait for laser scan \n');
    reset(r);
    
    while ~killNode
        % Add the laser scan to the point cloud when received.
        if newSonarMsg
            mapper.add2PtCloud(sonarSub.LatestMessage, poseSub);
            newSonarMsg = false;
        end
        waitfor(r);
    end
end

function sonarCallback(src, msg)
    global newSonarMsg;
    newSonarMsg = true;
end
