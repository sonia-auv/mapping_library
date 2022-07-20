classdef HistogramFilter < PointCloudFilter
    % General filter for most use case
    
    properties
        % ROS Parameters
        param % int nbin
    end
    
    methods
        function obj = HistogramFilter(param)
            obj.param = param;
        end
        
        % Filter using histogram bins egdes, Only work for one column for 1
        % column at this time, use plot = true to plot the histogram.
        % Return the fitered Pointcloud
        function filteredPT = filter(this, bundle, column, plot)

            if nargin > 3
              debug = plot;
            else
              debug = false;
            end

            if(debug)
                hist = histogram(bundle(2:end,column),this.param.nBin); %remove 0,0,0
                edges = hist.BinEdges;
            else
                [~, edges] = histcounts(bundle(:,column),this.param.nBin);
            end
            
            logic = bundle(:,column) > edges(this.param.nBinsToFilterOut + 1);
            filteredPT = bundle(logic,:);

        end
    end
end