function [exps_return] = exps(w,q,theta)
%UNTITLED21 此处显示有关此函数的摘要
%   此处显示详细说明
v=-cross(w,q);
s=[v;w];
expw=angvec2r(theta,w);
exps_return=[expw,(eye(3)-expw)*cross(w,v)+w*w'*v*theta;0,0,0,1];
end

