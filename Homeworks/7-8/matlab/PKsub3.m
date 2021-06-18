function [theta] = PKsub3(w,r,p,q,delta)
%   exp(sh*theta)*p-q=delta
%   w转轴 r转轴上一点 p初始点 q终止点 delta旋转后距离q的距离
u=p-r;
v=q-r;
ud=u-w*w'*u;
vd=v-w*w'*v;
ud_abs = sqrt(ud'*ud);
vd_abs = sqrt(vd'*vd);
deltad_square = delta^2-(w'*(p-q))'*(w'*(p-q));

theta0=atan2(w'*cross(ud,vd),ud'*vd);
Phi = acos((ud_abs^2+vd_abs^2-deltad_square)/(2*ud_abs*vd_abs));
theta=theta0+Phi;
% 存在0，1，2解
% theta=theta0-Phi;
end

