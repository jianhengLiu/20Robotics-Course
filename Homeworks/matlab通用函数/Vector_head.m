function [vectorhead] = Vector_head(vector)
%UNTITLED9 此处显示有关此函数的摘要
%   此处显示详细说明
vectorhead = [0,-vector(3),vector(2);
                vector(3),0,-vector(1);
                -vector(2),vector(1),0;];
end

