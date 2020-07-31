function path = path_gen(W,RL,r)
% route = [ 1 2 7 9 4 6 11 8 3 5 10];

N = 5;
dx = 0.1;

x(1,1) = 0;
y(1,1) = 0;

x(1,2*N +2) = 0;
y(1,2*N +2) = 0;

for i = 2 : N+1
    x(1,i) = 20 + W*(i-2);
    y(1,i) = 20;
end

for i = N+2 : 2*N +1
    x(1,i) = 20 + W*(i-7);
    y(1,i) = 40;
end

pathx = [x(1,1)];
pathy = [y(1,1)];

% path into field 
dist = 2;
% go up 
        i=0.1:dx:dist;
        x_cord = x(1,1) + i;
        y_cord = y(1,1) + i ;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];

% go right 

X_value = 20 - r - dist;

   i=0.1:dx:X_value;
        x_cord = x(1,1) + dist + i;
        y_cord = y(1,1) + dist + i*0 ;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
    
    % turn into path of first row
    
       i=3*pi/2:dx:2*pi;
        x_cord = x(1,1) + X_value + dist + r*cos(i);
        y_cord = y(1,1) + dist+ r + r*sin(i) ;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];

     % go straight through row 1 - south - north
        
        i=0.1:dx: (2*RL - r -2);
        x_cord = x(1,2) + i*0;
        y_cord = y(1,1) + r +dist + i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
     % turn into row 3 - north side
        dist = x(1,9) - x(1,7);
        X_value = dist - 2*r;
        
        i=pi:-dx:pi/2;
        x_cord = x(1,7) + r + r*cos(i);
        y_cord = y(1,7) + r*sin(i) ;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=0.1:dx:X_value;
        x_cord = x(1,7) + r + i;
        y_cord = y(1,7) + r + i*0 ;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=pi/2:-dx:0;
        x_cord = x(1,7) + r + X_value + r*cos(i);
        y_cord = y(1,7) + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
      % stright through row 3 - north to south
        i=0.1:dx: RL;
        x_cord = x(1,9) + i*0;
        y_cord = y(1,9) - i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
      % turn into row 5 - south side
      
      dist = x(1,6) - x(1,4);
      X_value = dist - 2*r;
      
        i=pi:dx: 3*pi/2;
        x_cord = x(1,4) + r + r*cos(i);
        y_cord = y(1,4)  + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=0:dx: X_value;
        x_cord = x(1,4) + r + i;
        y_cord = y(1,4)  - r + i*0;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=3*pi/2:dx: 2*pi;
        x_cord = x(1,4) + r + X_value + r*cos(i);
        y_cord = y(1,4)  + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
       % go straight through row 5 south to north
       i=0.1:dx: RL;
        x_cord = x(1,6) + i*0;
        y_cord = y(1,6) + i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        % turn into row 2 - north side
        
        dist = x(1,11) - x(1,8);
        X_value = dist - 2*r; 
        
        i=0:dx: pi/2;
        x_cord = x(1,11) - r + r*cos(i);
        y_cord = y(1,11)  + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=0:dx: X_value;
        x_cord = x(1,11) - r -i;
        y_cord = y(1,11)  + r + i*0;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
       
        i=pi/2:dx: pi;
        x_cord = x(1,11) - r - X_value + r*cos(i);
        y_cord = y(1,11) + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        % go down row 2 north - south
        
        i=0:dx: RL;
        x_cord = x(1,8) + i*0;
        y_cord = y(1,8)  -i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        % turn into row 4- south side
        dist = x(1,5) - x(1,3);
        X_value = dist - 2*r;
      
        i=pi:dx: 3*pi/2;
        x_cord = x(1,3) + r + r*cos(i);
        y_cord = y(1,3)  + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=0:dx: X_value;
        x_cord = x(1,3) + r + i;
        y_cord = y(1,3)  - r + i*0;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=3*pi/2:dx: 2*pi;
        x_cord = x(1,3) + r + X_value + r*cos(i);
        y_cord = y(1,3)  + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        % go straight through row 4 - south to north
        
        i=0:dx: RL;
        x_cord = x(1,5) + i*0;
        y_cord = y(1,5)  +i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        % turn into row 0 - north to south
        
        dist = x(1,5) - x(1,2) + W; 
        X_value = dist - 2*r;
        
        i=0:dx: pi/2;
        x_cord = x(1,10) - r + r*cos(i);
        y_cord = y(1,10)  + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=0:dx: X_value;
        x_cord = x(1,10) - r -i;
        y_cord = y(1,10)  + r + i*0;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
       
        i=pi/2:dx: pi;
        x_cord = x(1,10) - r - X_value + r*cos(i);
        y_cord = y(1,10) + r*sin(i);
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        % go down row 2 north - south
        
        i=0:dx: RL;
        x_cord = x(1,2) - W + i*0;
        y_cord = y(1,7)  - i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        
        % return back to origin 
        dist = 2
        
        i=0:dx: (RL - r - dist);
        x_cord = x(1,2) - W + i*0;
        y_cord = y(1,2)  - i;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];
        
        i=2*pi:-dx: 3*pi/2;
        x_cord = x(1,2) - W + - r + r*cos(i);
        y_cord = y(1,1) + r + dist + r*sin(i) ;
        pathx = [pathx x_cord];
        pathy = [pathy y_cord];  
        
    path = [pathx ; pathy];
end
      





