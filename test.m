function proc_mapping

    bag = rosbag('2022-04-30-CVM-SONAR.bag');
    bagSelection = select(bag, 'Topic', '/provider_sonar/LaserScan');
    msgStructs = readMessages(bagSelection,'DataFormat','struct');
    
    % scan = rosReadLidarScan(msgStructs{1}); 
    % 
    % xyzCoord = [scan.Cartesian(:,1), scan.Cartesian(:,2), zeros(size(scan.Cartesian(:,1)))];
    % 
    % ptCloud = pointCloud(xyzCoord);
    % 
    % map3D = occupancyMap3D(10);
    % insertPointCloud(map3D,[0,0,0,1,0,0,0],ptCloud,10)
    % show(map3D)
    
    % disp(ptCloud);
    
    %pcshow(ptCloud)
    
    map3D = occupancyMap3D(10);
    for i=1:numel(msgStructs)
        %disp(msgStructs{i})
        %rosPlot(msgStructs{i});
        scan = rosReadLidarScan(msgStructs{i}); 
        % scatter(scan.Cartesian(:,2), scan.Cartesian(:,1));
        xyzCoord = [scan.Cartesian(:,1), scan.Cartesian(:,2), (i/10) * ones(size(scan.Cartesian(:,1)))];
        
%         xyzPoint = sonar2NED(pos, quat, [0,0,0], [scan.Cartesian(:,1), scan.Cartesian(:,2), zeros(size(scan.Cartesian(:,1)))]);
%     
        ptCloud = pointCloud(xyzCoord);
%        
        insertPointCloud(map3D,[0,0,0,1,0,0,0],ptCloud,50)
        show(map3D)
        pause(0.01);
       disp(i);
    end
end