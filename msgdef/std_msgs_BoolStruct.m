function msg = std_msgs_BoolStruct
% Message struct definition for std_msgs/Bool
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/Bool',...
    'Data',ros.internal.ros.messages.ros.default_type('logical',1));
coder.cstructname(msg,'std_msgs_BoolStruct_T');
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
