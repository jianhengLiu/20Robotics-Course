function [theta] = PKsub1(w,r,p,q)
%   exp(sh*theta)*p=q
%   w转轴 r转轴上一点 p初始点 q终止点
u=p-r;
v=q-r;
ud=u-w*w'*u;
vd=v-w*w'*v;
theta=atan2(w'*cross(ud,vd),ud'*vd);
% theta=theta/pi*180;
end

