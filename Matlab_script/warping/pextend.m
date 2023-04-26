function X = pextend(x)
%PEXTEND Convert to homogeneous coordinates.
X = [x;ones(1,size(x,2))];
end

