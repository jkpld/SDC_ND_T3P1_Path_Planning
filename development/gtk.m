function [collide, distance] = gtk(shape1, shape2, computeDistance)

distance = 0;
simplex = [];

shape1(:,3) = 0;
shape2(:,3) = 0;

d = [1,0,0];
% d = mean(shape1 - shape2);
s = support(shape1,shape2,d);
d = -s;

simplex(1,:) = s;

counter = 1;
collide = false;
while true
   
    if counter > 56
        break;
    end
    
    a = support(shape1,shape2,d);
    simplex(end+1,:) = a;
    
    if a*d' < 0
        collide = false;
        break;
    end
    
    [collide, simplex, d] = doSimplex(simplex, d);
    
    if collide
        break
    end
    
    counter = counter + 1;
end

if ~collide && computeDistance
    
    if size(simplex,1)==3
        p1 = closestPoint_to_origin(simplex(3,:),simplex(2,:));
        p2 = closestPoint_to_origin(simplex(3,:),simplex(1,:));
        
        if (p1*p1' < p2*p2')
            simplex(1,:) = [];
            d = -p1;
        else
            simplex(2,:) = [];
            d = -p2;
        end
    else
        d = -closestPoint_to_origin(simplex(2,:),simplex(1,:));
    end
        
    counter = 1;
    while true
        
        if counter > 56
            break;
        end
        
        if d*d' == 0
            distance = 0;
            break;
        end
        
        a = support(shape1, shape2, d);
        simplex(end+1,:) = a;

        if (simplex(3,:)-simplex(2,:))*d' < 0.002
            distance = sqrt(d*d');
            break;
        end
        
        p1 = closestPoint_to_origin(simplex(3,:),simplex(2,:));
        p2 = closestPoint_to_origin(simplex(3,:),simplex(1,:));
        
        if (p1*p1' < p2*p2')
            simplex(1,:) = [];
            d = -p1;
        else
            simplex(2,:) = [];
            d = -p2;
        end
        counter = counter + 1;
    end
end

end

function [collide, simplex, d] = doSimplex(simplex, d)

collide = false;

switch size(simplex,1)
    case 2
        ab = simplex(1,:) - simplex(2,:);
        a0 = -simplex(2,:);
        
        if a0*ab' > 0
            d = cross(cross(ab,a0),ab);
        else
            simplex(1,:) = [];
            d = a0;
        end
    case 3
        ab = simplex(2,:) - simplex(3,:);
        ac = simplex(1,:) - simplex(3,:);
        abc = cross(ab,ac);
        a0 = -simplex(3,:);
        
        if cross(abc,ac)*a0' > 0
            if ac*a0' > 0
                simplex(2,:) = [];
                d = cross(cross(ac,a0),ac);
            else
                if ab*a0' > 0
                    simplex(1,:) = [];
                    d = cross(cross(ab,a0),ab);
                else
                    simplex(1:2,:) = [];
                    d = a0;
                end
            end
        else
            if cross(ab,abc)*a0' > 0
                if ab*a0' > 0
                    simplex(1,:) = [];
                    d = cross(cross(ab,a0),ab);
                else
                    simplex(1:2,:) = [];
                    d = a0;
                end
            else
                collide = true;
            end
        end
end
end

function d = support(shape1, shape2, d)
[~,i1] = max(shape1*d.');
[~,i2] = max(-shape2*d.');
d = shape1(i1,:) - shape2(i2,:);
end

function p = closestPoint_to_origin(a,b)
ab = b-a;

if ab*ab' < 0.002
    p = a;
    return;
end
% u = a + t*ab; % t in [0,1]
% d = (a+t*ab)*(a+t*ab)'
% 0 = 2*(a + t*ab)*ab'
t = -(a*ab')/(ab*ab');
t = min(max(t,0),1);
p = a + t*ab;
end

% function c = cross(a,b)
% c = [a(2).*b(3)-a(3).*b(2);
%         a(3).*b(1)-a(1).*b(3);
%         a(1).*b(2)-a(2).*b(1)];
%     
% if ~iscolumn(a) || ~iscolumn(b)
%     c = c.';
% end    
% end