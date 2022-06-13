function lidarMsgOut = packagePointCloud(XYZ, RGB)

    lidarMsgOut = rosmessage('sensor_msgs/PointCloud2',"DataFormat","struct");
    lidarMsgOut.IsBigendian = false;
    lidarMsgOut.IsDense = true; 
    lidarMsgOut.Header.FrameId = 'BODY';
    % Calculate number of points
    numPts = size(XYZ,1);
    
    % Assign metadata
    lidarMsgOut.Height = uint32(1);
    lidarMsgOut.Width = uint32(numPts);
    lidarMsgOut.PointStep = uint32(16); 
    lidarMsgOut.RowStep = uint32(16 * numPts);

    % Assign point field data
    fieldNames = {'x','y','z','intensity'};
    lidarMsgOut.Data = zeros(lidarMsgOut.RowStep,1,'uint8');
    for idx = 1:4
        lidarMsgOut.Fields(idx) = rosmessage('sensor_msgs/PointField',"DataFormat","struct");
        fName = fieldNames{idx};
        lidarMsgOut.Fields(idx).Name(1:numel(fName)) = uint8(fName);
        lidarMsgOut.Fields(idx).Datatype = uint8(7);
        lidarMsgOut.Fields(idx).Count = uint32(1);
    end

    lidarMsgOut.Fields(1).Offset = uint32(0);
    lidarMsgOut.Fields(2).Offset = uint32(4);
    lidarMsgOut.Fields(3).Offset = uint32(8);
    lidarMsgOut.Fields(4).Offset = uint32(12);
    
    % Assign raw point cloud data in uint8 format
    for idx = 1:numPts
       startIdx = (idx-1)*lidarMsgOut.PointStep + 1;
       lidarMsgOut.Data(startIdx:startIdx+11) = typecast(XYZ(idx,:), 'uint8');
       lidarMsgOut.Data(startIdx+12:startIdx+15) = typecast(RGB(idx,:), 'uint8');
       % NOTE: The 16th byte remains empty
    end

end
