function proc_mapping

    if coder.target('MATLAB')
        setenv("AUV","AUV8");
        clear all;
        if ~ ros.internal.Global.isNodeActive
            % partir le node ros matlab 
            rosinit;
        end
        system("rosparam load ./config/AUV8.yaml");
        fprintf('INFO : proc mapping : Ros param loaded. \n');
    end

    % Variables
    rosSpin = 20;
    r = rosrate(rosSpin);   
    % Proc_mapping startup
    rosNode = RosNode(rosSpin);
    rosNode.spin(r);
end
