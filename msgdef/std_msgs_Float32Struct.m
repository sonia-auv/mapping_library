function msg = std_msgs_Float32Struct
% Message struct definition for std_msgs/Float32
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/Float32',...
    'Data',ros.internal.ros.messages.ros.default_type('single',1));
coder.cstructname(msg,'std_msgs_Float32Struct_T');
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
