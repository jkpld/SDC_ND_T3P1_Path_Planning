fiasdfasdfasdfclc
rec = [-1,-1;
       -1, 1;
        1, 1;
        1,-1]';
    
rot = @(th) [cosd(th), sind(th); -sind(th), cosd(th)];

rec1 = rot(45)*(rec.*[1;4]) + [3;7];
rec2 = rot(0)*(rec.*[3;1]) + [1;1];

rec1 = rec1.';
rec2 = rec2.';

figure(1)
clf(1)
patch('Vertices',rec1, 'Faces',1:4,'facecolor','none','edgecolor','b')
patch('Vertices',rec2, 'Faces',1:4,'facecolor','none','edgecolor','r')

daspect([1 1 1]);


tic
profile on
for i = 1:50
collide1 = RectangleCollision(rec1, rec2);
[collide2, distance] = gtk(rec1,rec2,false);
end

profile off
profile viewer
toc

% distance


fn1 = @() RectangleCollision(rec1, rec2);
fn2 = @() gtk(rec1,rec2,false);

timeit(fn1,1)
timeit(fn2,2)