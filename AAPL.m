function apply = AAPL(type, V, Polygon, i, j)
%AAPL returns true or false for appilcability condition.
% type - "A" or "B" type applicability condition
% V - the edge vectors of the "type"
% Poly - the vertices of the polygon of the other than "type"
% i - for A vertices, j - for B vertices

% extend the Polygon and edge Vector to access i,j -+ 1 elements.
ExPoly = [Polygon(end, :); Polygon; Polygon(1, :)];

if type=="A"
    vA_i = V(i,:);
    % take j-1 and j, but make sure there is no index error in j-1=0
    Db_jp = ExPoly(j, :) - Polygon(j, :); % j-1 is like taking 4 1 2 3 by order of 1 2 3 4
    % take j+1 and j, but make sure there is no error in j+1=5
    Db_jm = ExPoly(j+2, :) - Polygon(j, :); % j+1 is like taking 2 3 4 1 by order of 1 2 3 4
    apply=false; % by default unless found otherwise
    if dot(vA_i, Db_jp)>=0
        if dot(vA_i, Db_jm)>=0
            apply=true;
        end
    end

elseif type=="B"
    vB_j = V(j,:);
    Da_ip = ExPoly(i, :) - Polygon(i, :);
    Da_jm = ExPoly(i+2, :) - Polygon(i, :);
    apply=false;
    if dot(vB_j, Da_ip)>=0
        if dot(vB_j, Da_jm)>=0
            apply=true;
        end
    end
end
end