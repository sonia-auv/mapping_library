classdef (Abstract) PointCloudFilter
    %POINTCLOUDFILTER classe mere
    
    properties
       
    end
  
    methods (Abstract, Access = public )
 
        filteredPT = filter(this,rawPT);

    end
end

