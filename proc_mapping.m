function proc_mapping
    if coder.target('MATLAB')
        setenv("AUV","AUV8");
        
        if ~ ros.internal.Global.isNodeActive
            % partir le node ros matlab 
            rosinit;
        end
    end

    % Variables
    rosSpin = 10;
    r = rosrate(rosSpin);   
    
    % Proc_mapping startup
    mapper = Mapper();
    mapper.spin(r);
end
