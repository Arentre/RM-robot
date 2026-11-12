function p = gimbal_read_axis_params(s,axis)
state = gimbal_usb_command(s,'GET','ack:GET,');
key = [lower(axis) 'p'];
fields = regexp(state,[',' key '=([^,]+,[^,]+,[^,]+,[^,]+,[^,]+),'], ...
    'tokens','once');
assert(~isempty(fields),'GET did not include %s axis parameters.',axis);
p = sscanf(fields{1},'%f,%f,%f,%f,%f').';
assert(numel(p) == 5 && all(isfinite(p)) && p(5) > 0 && p(5) <= 1, ...
    'Invalid %s axis parameters.',axis);
end
