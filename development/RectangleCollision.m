function collide = RectangleCollision( rect1, rect2) %#codegen

% All possible normal vectors of rectangles  (4 of them)
n = [diff(rect1(1:3,:),1); diff(rect2(1:3,:),1)];
n = [-n(:,2),n(:,1)];

% project rect1 along normals
p1 = rect1*n'; % 4 x 4 , each column is the project of the 4 vertices along a specific normal
p2 = rect2*n'; % 4 x 4 , each column is the project of the 4 vertices along a specific normal

% must check if the minimum of one shape is larger than the maximum of
% another
p1min = min(p1);
p2max = max(p2);

if any(p1min > p2max)
    collide = false;
else
    p1max = max(p1);
    p2min = min(p2);
    if any(p2min > p1max)
        collide = false;
    else
        collide = true;
    end
end