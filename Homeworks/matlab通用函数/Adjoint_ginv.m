function [Adj_inv] = Adjoint_ginv(g)
%   求取伴随矩阵
g=mat2cell(g,[3,1],[3,1]);
R=g{1,1};
p=g{1,2};
p_head = Vector_head(p);
Adj_inv = [R',           -R'*p_head;
       zeros(3),     R'];
end

