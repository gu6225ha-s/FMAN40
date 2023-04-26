function area = polyarea(P)
%POLYAREA Compute the signed area of the polygon P.
N = size(P,2);
area = 0;
for i = 1:N
    j = 1+mod(i,N);
    area = area+P(1,i)*P(2,j)-P(1,j)*P(2,i);
end
end