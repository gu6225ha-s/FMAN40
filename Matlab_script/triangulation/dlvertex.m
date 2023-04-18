classdef dlvertex < handle
    %DLVERTEX Doubly linked vertex
    
    properties
        Position % Coordinates of the vertex
        Index % Index of the vertex
        Prev = dlvertex.empty % Previous vertex
        Next = dlvertex.empty % Next vertex
    end

    methods
        function vert = dlvertex(position,index)
            %DLVERTEX Create a new vertex
            vert.Position = position;
            vert.Index = index;
        end

        function remove(vert)
            %REMOVE Remove a vertex from the doubly linked list
            if ~isempty(vert.Prev)
                vert.Prev.Next = vert.Next;
            end
            if ~isempty(vert.Next)
                vert.Next.Prev = vert.Prev;
            end
            vert.Prev = dlvertex.empty;
            vert.Next = dlvertex.empty;
        end
    end
end