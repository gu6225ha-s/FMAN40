function I = tripoly(P)
%TRIPOLY Triangulate a polygon using ear clipping.
% I = tripoly(P) triangulates a simple, counter-clockwise oriented polygon
% P (a Nx2 matrix) using naive ear clipping, returning the list of
% triangles I (a (N-2)x3 matrix).

N = size(P,1);
if N < 3
    error('Polygon must have at least three points.');
end

% Create linked list of vertices
head = dlvertex(P(1,:),1);
tail = head;
for i = 2:N
    vert = dlvertex(P(i,:),i);
    tail.Next = vert;
    vert.Prev = tail;
    tail = vert;
end
tail.Next = head;
head.Prev = tail;

% Triangulate
I = zeros(N-2,3);
i = 1;
while N >= 3
    [I(i,:),head] = getear(head);
    i = i+1;
    N = N-1;
end
end

function [ear,vert] = getear(vert)
%GETEAR Find one "ear" of the polygon and remove it.
v = vert;
while true
    if isear(v)
        ear = [v.Prev.Index v.Index, v.Next.Index];
        vert = v.Next;
        v.remove();
        return
    end
    v = v.Next;
    if v == vert
        break
    end
end
error('Failed to find ear.');
end

function ear = isear(vert)
%ISEAR Check if (vert.Prev,vert,vert.Next) is an "ear".
if isconvex(vert.Prev.Position,vert.Position,vert.Next.Position)
    ear = true;
    v = vert.Next.Next;
    while v ~= vert.Prev
        if intriangle(vert.Prev.Position,vert.Position,vert.Next.Position,v.Position)
            ear = false;
            break;
        end
        v = v.Next;
    end
else
    ear = false;
end
end

function area = triarea(a,b,c)
%TRIAREA Calculate the signed area (multiplied by two) of triangle (a,b,c).
area = a(1)*(b(2)-c(2))+b(1)*(c(2)-a(2))+c(1)*(a(2)-b(2));
end

function convex = isconvex(a,b,c)
%ISCONVEX Check if b is on the right of (or on) the line segment ac.
convex = triarea(a,b,c)>=0;
end

function inside = intriangle(a,b,c,p)
%INTRIANGLE Check if point p is inside the triangle (a,b,c).
a1 = triarea(p,a,b);
a2 = triarea(p,b,c);
a3 = triarea(p,c,a);
neg = a1<0||a2<0||a3<0;
pos = a1>0||a2>0||a3>0;
inside = ~(neg&&pos);
end