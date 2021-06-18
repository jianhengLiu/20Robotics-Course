function [Adj] = Adjoint_g(g)
%   求取伴随矩阵
g=mat2cell(g,[3,1],[3,1]);
R=g(1,1);
p=g(1,2);
p_head = Vector_head(p);
Adj = [R,   p_head*R;
       0,   R];
end

