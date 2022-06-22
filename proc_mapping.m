function proc_mapping
    if coder.target('MATLAB')
        setenv("AUV","AUV8");
        clear all;
        if ~ ros.internal.Global.isNodeActive
            % partir le node ros matlab 
            rosinit;
        end
    end

    % Variables
    rosSpin = 20;
    r = rosrate(rosSpin);   
    % Proc_mapping startup
    rosNode = RosNode();
    rosNode.spin(r);
end
