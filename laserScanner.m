% function assumes a laser scanner with a pose in world coordinates defined by Tl. It shoots rays from
% -angleSpan/2 to +angleSpan/2 with step angleStep.
% Given a map (occupancy grid with obstacles) with origin at (0,0) and uper corner (Xmax, Ymax),
% the result is all the ranges from obstacles. It returns Inf, when no
% obstacles within maxRange. In real lidars this value varies, depending on
% the manufacturer.
function p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax)
N = round(angleSpan/angleStep +1); %number of scan points
p = zeros(N, 2);
[R, C] = size(map);
i=1;

DEBUG = 0; % set to 1 to see laser line on the map
%transform laser origin to world frame
P1 = Tl*[0 0 1]';
x1=P1(1);     y1=P1(2);
[ I1, J1 ] = XYtoIJ(x1, y1, Xmax, Ymax, R, C); % find the corresponding pixel
for a = angleSpan/2 : - angleStep: - angleSpan/2
    %first produce target point for laser in scanner frame
    Xl = rangeMax * cos(a);
    Yl = rangeMax * sin(a);
    
    %Transform target point in world frame
    P2 = Tl*[Xl Yl 1]';
    x2=P2(1); y2=P2(2);
    
    %clip line to world boundary polygon
    edge = clipLine([x1,y1,x2-x1,y2-y1],[0 Xmax 0 Ymax]);
    %assume laser origin is always inside map
    x2 = edge(3); y2 = edge(4);
    
    % now compute if ray intersects an obstacle in the map
    % first map world points to integer coordninates; row I: 1...R, column J
    % 1..C
    
    [ I2, J2 ] = XYtoIJ(x2, y2, Xmax, Ymax, R, C);
    
    p(i,1)=a; %the angle
    %call laser range function from rvctools/common
    Pxel = laserRange([I1 J1], [I2 J2], map); %returns obstacle pixel; if no obstacle returns inf
    Io=Pxel(1); Jo=Pxel(2);
    
    if (isinf(Io) || isinf(Jo))
        p(i,2)=Inf; %arbitrarily assign maximum range
    else
        %compute target point (that gave reflection) in world coordinates
        [ xTarget, yTarget ] = IJtoXY(Io, Jo, Xmax, Ymax, R, C);
        % compute distance from laser origin to target point
        r = sqrt((x1 - xTarget)^2 + (y1-yTarget)^2);
        p(i,2) = r;
        
        
        % for debugging, show scan line
        if DEBUG == 1
            l=bresenhamFast(I1,J1,Io,Jo);
            for k=1:length(l)
                map(l(k,1),l(k,2))=1;
            end
            imagesc(map);
            for k=1:length(l)
                map(l(k,1),l(k,2))=0;
            end
        end
        
        
    end
    
    
    
    % shoot next laser beam
    i = i+1;
end

end

