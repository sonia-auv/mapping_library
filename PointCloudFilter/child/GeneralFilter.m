classdef GeneralFilter < PointCloudFilter
    % General filter for most use case
    
    properties
    a;
    end
    
    methods
        function obj = GeneralFilter()
            
        end
        
        function filteredPT = filter(this,rawPT)
            
            filteredPT = pcdenoise(pcdownsample(rawPT, 'gridAverage', 0.01));

        end
    end
end

