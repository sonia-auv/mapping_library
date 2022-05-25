function proc_mapping
    % Global variables
    global newSonarMsg newSonarPtMsg bundleStarted;
    newSonarMsg = false;
    newSonarPtMsg = false;
    bundleStarted = false;
    
    % Variables
    rosSpin = 10;
    r = rosrate(rosSpin);
    killNode = false;     
 
    % Subscribers
    poseSub = rossubscriber('/proc_nav/auv_pose', 'geometry_msgs/Pose', "DataFormat", "struct");
    startStopSub = rossubscriber('/proc_mapping/start_stop', 'std_msgs/Bool', @startStopCallback, "DataFormat", "struct");
    sonarSub = rossubscriber('/provider_sonar/point_cloud2', 'sensor_msgs/PointCloud2', @sonarCallback, "DataFormat", "struct");
    
    mapper = Mapper();
    
    fprintf('INFO : proc mapping : Node is started \n');
    fprintf('INFO : proc mapping : Wait for laser scan \n');
    reset(r);
    
    while ~killNode
        % Add the laser scan to the point cloud when received.
        if newSonarMsg && bundleStarted
            mapper.add2PtCloud(sonarSub.LatestMessage, poseSub);
            newSonarMsg = false;
        end
        waitfor(r);
    end
end

function startStopCallback(src, msg)
    global bundleStarted;
    bundleStarted = msg.Data;
    if bundleStarted
        fprintf('INFO : proc mapping : Bundle record started \n');
    else
        fprintf('INFO : proc mapping : Bundle record stopped \n');
    end
end

function sonarCallback(src, msg)
    global newSonarMsg;
    newSonarMsg = true;
end
