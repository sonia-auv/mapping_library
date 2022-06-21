function msg = std_msgs_HeaderStruct
% Message struct definition for std_msgs/Header
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/Header',...
    'Seq',ros.internal.ros.messages.ros.default_type('uint32',1),...
    'Stamp',ros_TimeStruct,...
    'FrameId',ros.internal.ros.messages.ros.char('string',0));
coder.cstructname(msg,'std_msgs_HeaderStruct_T');
coder.varsize('msg.FrameId',[1 1000000000],[0 1]);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
