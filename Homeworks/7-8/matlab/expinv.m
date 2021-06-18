function [exps_inv] = expinv(exps)
exps_cell=mat2cell(exps,[3,1],[3,1]);
exps_inv=[exps_cell{1,1}',-exps_cell{1,1}'*exps_cell{1,2};0,0,0,1];
end

