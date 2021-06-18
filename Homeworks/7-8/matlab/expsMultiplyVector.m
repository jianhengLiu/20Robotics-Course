function [vector_return] = expsMultiplyVector(exps,vector)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
vector_ho=[vector;1];
vector_return_ho=mat2cell(exps*vector_ho,[3,1],[1]);
vector_return=vector_return_ho{1,1};
end

