classdef GeneralFilter < PointCloudFilter
    % General filter for most use case
    
    properties

    end
    
    methods
        function obj = GeneralFilter()
            
        end
        
        function filteredPT = filter(this, bundle)
            % Create the point cloud and apply denoise filter plus a downsample.
            rawPT = pointCloud(bundle(:, 1:3), 'Intensity', bundle(:, 4));
            filteredPT = pcdenoise(pcdownsample(rawPT, 'gridAverage', 0.01));
        end
    end
end

