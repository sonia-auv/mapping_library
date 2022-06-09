classdef (Abstract) PointCloudFilter
    %POINTCLOUDFILTER classe mere
    
    properties
       
    end
  
    methods (Abstract, Access = public )
 
        filteredPT = filter(this, bundle);

    end

    methods
        function ptShow(this, ptCloud)
            if coder.target('MATLAB')
                pcshow(ptCloud);
            end
        end
    end
end

