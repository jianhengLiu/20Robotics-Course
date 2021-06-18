function [Rotx] = rotx_sym(theta)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
Rotx = [1,            0,            0;
       0,            cos(theta), -sin(theta);
       0,            sin(theta), cos(theta)];
end

