function [ksi] = lidaishu(w,q)
%   求取李代数
%   此处显示详细说明
v = -cross(w,q);
ksi=[v;w];
end

