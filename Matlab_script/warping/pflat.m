function y = pflat(x)
%PFLAT Divide points in homogeneous coordinates with their last coordinate
y = x./repmat(x(end,:),[size(x,1) 1]);
end