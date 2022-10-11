function [control] = pointMassCtrl(x,v,xd,param)
ref_pos = xd(:,1);
ref_vel = xd(:,2);
ref_acc = xd(:,3);

error_pos = x - ref_pos;
error_vel = v - ref_vel;

control = -param.kp*error_pos - param.kv*error_vel + ref_acc;
end