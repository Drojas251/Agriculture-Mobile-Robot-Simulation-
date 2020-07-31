%% Generte Nursey
% get ground truth bitmap 'bitmap' from the generateNursery script
% assign the bitmap to the variable 'map'
% true_tree_location is a global variable created in generateNursery that
% gets the location and radius of the tree tunks when randomly creating it
% in generateNursey. 
% NOTE: if a different generateNursey function is used other than the one
% in the script, then true_tree_location must be initialized as in
% generatNursery in order for the error statistics to be performed. 

addpath('./lib');

global bitmap true_tree_location; 
generateNursery()
map = bitmap; % variable map holds the ground truth bitmap

%% **********Laser Variables

global Bitmap; % The bitmap that will be generated with laser scanning measurments 
global rangeMax; % range of laser

rangeMax = 50; % meters
angleSpan = pi; angleStep = angleSpan/360; % span of laser scanner
Xmax = 42; Ymax = 48; %physical dimensions of space (m)
X_max_nurs = 42; Y_max_nurs = 42; %physical dimensions of ground truth space (m)

R = 500; C = 500; %rows and columns; discretization of physical space in a grid
Bitmap = 0.5* ones(R, C); %initialize as empty
global odd_bitmap
odd_bitmap=ones(R, C);

%% *****  Path planning *****

NR = 5; % number of rows
NC = 7;
W = 3; % width of rows
RL = 20; % Row length
L=3;  % wheel base 
gamma = 55*pi/180; % max steering angle
r = L/(tan(gamma)); % turning radius

% generate path
path = path_gen(W,RL,r); 
one = ones(1,length(path(1,:)));
path = [path ; one]; % path in homo coordinates
%% Control loop set up
global dT; %integration interval
global DT; %control interval
global epsilon_dist; %distance from goal point considered close enough to stop robot (goal reached)
global epsilon_speed; %almost zero speed

epsilon_dist = 0.1; % (m)
epsilon_speed = 0.02; % (m/s)
dT = 0.01; % (s) model integration step
DT =  0.1; % (s) controller integration step
T = 200; % control loop simulation time

NUM = T/DT; % number of time steps

% define state vectors
q_true = zeros(5,NUM); % state vector over NUM time steps
u=zeros(2, NUM); % input vectors over NUM time steps

% initial Pose 
x_initial = 0;
y_initial = 0;
theta_initial = pi/2;

% initialize initial state
q_true(1,1) = x_initial;
q_true(2,1) = y_initial;
q_true(3,1) = theta_initial;

gammaMax = gamma; % max steering angle
Vmax =1; % max velocity 

% Control constraints
Umax=[gammaMax Vmax]'; % max / min inputs  
Umin= - Umax;
STEER_INPUT = 1; SPEED_INPUT=2; % index for input vector 'u'

% state constraints
Qmax(1) = Inf; Qmax(2) = Inf; Qmax(3) = Inf;
Qmax(4) = gammaMax; Qmax(5) = Vmax;
Qmin = -Qmax; % symmetrical negative constraints for minimum values

tau_gamma = 0.1; % steering time lag (s)
tau_v = 0.2; % velocity time lag (s)

% *** controller variables **
Ld = 0.8; % look ahead distance
m = 1; % control variable used to control the GPS readings
start_ind=1;  % path start index
end_ind=300; % path end index
psi=1; % path start index
dif_dist=100; 


% EKF variables

X = zeros(3, NUM); % estimated state vector
% initialize
X(1,1) = x_initial;
x(2,1) = y_initial;
X(3,1) = theta_initial;

Pose = [x_initial, y_initial, theta_initial]; % initialize the first pose for laser scanning

% these are precomputed values 
 V = diag([0.02, 0.5*pi/180].^2);  %odemetry Covariance matrix
 W = diag([0.03, 0.029, 0.020].^2);  %sensor COvariance matrix
 Hx = eye(3);% jacobian for observation function
 Hw = eye(3);% jacobian for observation function
 P = zeros(3); %asumption of uncertainty covariance matrix. Set to zero becasue the true initial position is known 

%% *********** Controller *****

k = 1;
for t = 0:DT:T - 2*DT

% path controller

if end_ind>length(path(1,:)); 
    end_ind=length(path(1,:));  
end
    
        temp_path = path(:,max(1,start_ind-20):end_ind);

     [comp(1),comp(2),target_point,min_d]=purePursuitController(q_true(:,k), L, Ld, temp_path);
     comp(4)=min_d(1);
     comp(5)=min_d(2); 
     
     ind1= find(path(1,:)==comp(4)); % finding the beging of the temporary path
     ind2= find(path(2,:)==comp(5));
     ind=find(ismember(ind1,ind2)==1);
     start_ind=ind1(ind);
     [b,I]=min(abs(psi-start_ind));
     psi=start_ind(I); % compaires with the previous start index
     start_ind=psi;
     end_ind=start_ind+300;
     endcheck=sqrt((min_d(1,1)-path(1,end)).^2 + (min_d(2,1)-path(2,end)).^2); % stops at a certain distance of the end point
     if start_ind==length(path(1,:))| (endcheck <0.2 & k>500)  ;break; end % end of the path
  
     if comp(1) < gammaMax
        u(STEER_INPUT,k) = comp(1);
     else
        u(STEER_INPUT,k) = gammaMax;
     end
     
     u(SPEED_INPUT,k) = 1;

    % get the next state (k+1)
    [q_true_next, odo] = robot_odo(q_true(:,k), u(:,k), Umin, Umax,Qmin, Qmax, L, tau_gamma, tau_v);
    q_true(:, k+1) = q_true_next;
 % **********

% **EKF**
delta_dist = odo(1);
delta_theta = odo(2);

% estimate state vector
x = X(1,k);
y = X(2,k);
theta = X(3,k);

% integrate forward using odemetry 
% Use the previous state (x,y,theta)(k) to find the next state
% (x,y,theta)(k+1)
X(1, k+1) = x + delta_dist*cos(theta);
X(2, k+1) = y + delta_dist*sin(theta);
X(3, k+1) = theta + delta_theta;
 
%jacobian of the odemetry model 
% needed for linearization
 Fx = [1 0 -delta_dist*sin(theta);
     0 1 delta_dist*cos(theta);
     0 0 1];
 Fv = [cos(theta) 0;
     sin(theta) 0; 
     0 1];
 
% Update estimate state uncertainty 
P_next = Fx*P*Fx' + Fv*V*Fv';
P = P_next;

% apply update with sensing one seconds = 10 iterations
if m == 10

% get sensed GPS/compass data
[x_noisy, y_noisy, theta_noisy]=GPS_CompassNoisy(q_true(1,k+1),q_true(2,k+1),q_true(3,k+1));

% calculate the inovation
vx = x_noisy - X(1,k+1);
vy = y_noisy - X(2,k+1);
vtheta = theta_noisy - X(3,k+1);
v = [vx;vy;vtheta];

% calculate the Kalman gain 
S = Hx*P_next*Hx' + Hw*W*Hw';
K = P_next*Hx'*inv(S);

% Get updated estimation using estimated state from forward integration and
% GPS/compass readings 
X_update = X(:,k+1) + K*v;
P_update = P_next - K*Hx*P_next;

% updated Pose
X(:,k+1) = X_update;
P = P_update;

% To ensure an accurate laser scan of the field, only estimated pose that
% are generated from the EKF using GPS/Compass data are used. These are
% saved into a vector 'Pose' and passed later to the laserscanning function
qcurrent = [X(1,k+1), X(2,k+1), X(3,k+1)];
Pose = vertcat(Pose,qcurrent);

%set m back to 1 to control the GPS/Compass reading
m =1;

end
% increment control variables 
   m = m+1 ; 
  k = k+1;  
end

%% ***** Laser Measurement ********
  % should be an odd number
adj_span=3; % filter span (number of pints will be used for getting the median) 

mapb = flipud(map);

% imagesc(Bitmap);

for k = 1: length(Pose)

     Tl = SE2([ Pose(k,1) Pose(k,2) Pose(k,3)]);
       hold on   
    axis equal
    
    [Ine(k), Jne(k)] = XYtoIJ( Tl.t(1),Tl.t(2) , Xmax, Ymax, R, C);
    plot( Tl.t(1),Tl.t(2),'r.');
    
    pr = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, mapb, Xmax, Ymax);  % Raw positions
    mfp=pr(:,2);%median filtered points- angles will stay the same
    p=pr;
    %Median Filter
    for i=1:length(pr)-adj_span 
        temp=pr(i:i+adj_span-1,2); % temporary points with the size of the defined span
        mfp(i+(adj_span-1)/2)=median(temp); % getting the median as the representative
    end
  % sap median filtered points
    % Low pass filter
    lpfp = smooth(mfp,0.01); % Low pass filtered points (smoothed) 
     p(:,2)=mfp; % filtered measurements replaced with the raw points
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);                                     % median and low pass filter
        % possible infinite range is handled inside the update function
        n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax); % 2
    end
    
    %online progress view
    figure(2)
    imagesc(Bitmap); 
    colorbar
    plot( Jne(1:k), Ine(1:k),'r.');
    pause(0.1)
     
  
end
temp_bit=odd_bitmap==1;
temp_bit=temp_bit*.5;

obst=odd_bitmap>1;
obst=obst*1;

empty=odd_bitmap<1;
empty=empty.*odd_bitmap;

tot_bit= temp_bit + obst + empty;

% final generated Bitmap of the Nursery
figure(3)
imagesc(tot_bit);
title('Generated Occupancy Grid of the Nursery')
colorbar
Bitmap= flipud(tot_bit);
%%  Image Processing 

Bitmap(1:20,:) = 0;
Bitmap(480:500,:) =0;
Bitmap(:,1:20) = 0;
Bitmap(:,480:500) = 0;

S = kcircle(1);
clean = iclose(Bitmap, S);

clean = iopen(clean, S);
BW = im2bw(clean, 0.4);
stats = regionprops(BW,'all');


figure(4)
% filter data
imshow(BW);
title('Processed Image')

figure(5)
title('Circles Fit to Tree Trunks: Diameter Estimation')
[centers, radii, metric] = imfindcircles(BW,[1 20]);
imshow(BW)
hold on
viscircles(centers, radii,'EdgeColor','r');

%% estimate diameters 

% This section finds the statistics of the detected tree trunks and
% organizes the data

rows_trees = 7;
column_trees = 5;
size = rows_trees*column_trees;

% get statistics
stats = regionprops(BW,'all');

minr = stats(1).Centroid(2);
maxr = stats(1).Centroid(2);
minc = stats(1).Centroid(1);
maxc = stats(1).Centroid(1);

Area_vec = zeros(length(stats),1);

for i = 1:length(stats)
    Area_vec(i,1) = stats(i).Area;
end
min_area = min(Area_vec) ;

j=1;
for i=1 : length(stats)
if stats(i).Area>(min_area -1)
    k(j)=stats(i).Area;
    
    if minr > stats(i).Centroid(2)
        minr=stats(i).Centroid(2);
    end
    if minc > stats(i).Centroid(1)
        minc=stats(i).Centroid(1);
    end
    if maxr < stats(i).Centroid(2)
        maxr=stats(i).Centroid(2);
    end
    if maxc < stats(i).Centroid(1)
        maxc=stats(i).Centroid(1);
    end
    

    j=j+1;
    objnum=j;
end 
end

n=1;
for kk=0:6
for i=1:j-1
    if stats(i).Centroid(2) > minr+(kk*((maxr-minr)/6))-(maxr-minr)/12
        ro(i)=kk+1;
    end
end
end

for kk=0:4
for i=1:j-1
    if stats(i).Centroid(1)> minc+(kk*((maxc-minc)/4))-(maxc-minc)/8
        co(i)=kk+1;
    end
end
end

k=0;
for i=1 : length(stats)
if stats(i).Area>(min_area-1)
    k=(ro(i)-1)*5+co(i);
      area(ro(i),co(i))=stats(i).Area;
     
      % converts detected centriods to x,y
      [x1(i),y1(i)] = IJtoXY(stats(i).Centroid(2),stats(i).Centroid(1),X_max_nurs,Y_max_nurs,R,C);
      
      Centroid_x(ro(i), co(i)) = x1(i);
      Centroid_y(ro(i), co(i)) = y1(i);
      
      minorAxis(ro(i), co(i)) = stats(i).MinorAxisLength;
      majorAxis(ro(i), co(i)) = stats(i).MajorAxisLength;
      
      % computes the diameters based on major and minor axis 
      Diameter(ro(i), co(i)) = (stats(i).MinorAxisLength + stats(i).MajorAxisLength)/2;
      
% text(stats(i).Centroid(1),stats(i).Centroid(2), ...
%         sprintf('%d',k), ...
%         'EdgeColor','b','Color','r');
%     hold on

end 
end



%Uses the pixel resolution to convert Diameter readings to x,y
resolution = Xmax/500;
Diameter = Diameter*resolution;
radius_final = Diameter/2; 

% put the sensed tree data in a size x 3 array to be compared for error
% statistics
Trees_sensed = zeros(size, 3);
index = 1;
for i = 1:column_trees
    for j = 1: rows_trees
        Trees_sensed(index, :) = [Centroid_x(j,i), Centroid_y(j,i), Diameter(j,i)];
        index = index +1;
    end
end


%% Output
% outputs the statistics of each row to a text file Outputs.txt
% outputs contain location of tree trunk (x,y) and its diameter
% Starts with the top row of the nursery 
file = fopen('Outputs.txt','w');
rows = 5;

fprintf(file,'1 \n');
for i = 1:rows
    if Diameter(1,i) ~= 0
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(1,i),Centroid_y(1,i),Diameter(1,i));
    end
end
fprintf(file,' \n');

fprintf(file,'2 \n');
for i = 1:rows
     if(Diameter(2,i) ~= 0)
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(2,i),Centroid_y(2,i),Diameter(2,i));
    end
end
fprintf(file,' \n');

fprintf(file,'3 \n');
for i = 1:rows
     if(Diameter(3,i) ~= 0)
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(3,i),Centroid_y(4,i),Diameter(5,i));
    end
end
fprintf(file,' \n');

fprintf(file,'4 \n');
for i = 1:rows
     if(Diameter(4,i) ~= 0)
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(4,i),Centroid_y(4,i),Diameter(4,i));
    end
end
fprintf(file,' \n');

fprintf(file,'5 \n');
for i = 1:rows
     if(Diameter(5,i) ~= 0)
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(5,i),Centroid_y(5,i),Diameter(5,i));
    end
end
fprintf(file,' \n');

fprintf(file,'6 \n');
for i = 1:rows
     if(Diameter(6,i) ~= 0)
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(6,i),Centroid_y(6,i),Diameter(6,i));
    end
end
fprintf(file,' \n');

fprintf(file,'7 \n');
for i = 1:rows
     if(Diameter(7,i) ~= 0)
    fprintf(file,'%d, %.2f, %.2f, %.2f\n',i,Centroid_x(7,i),Centroid_y(7,i),Diameter(7 ,i));
    end
end
fprintf(file,' \n');



%% errors

% this section compares the actual tree data ' true_tree_location ' from
% the generateNursery function with the detected tree data 'Tree_sensed'
% that was outputted to the output file. 
dist_errors = [];
diam_errors = [];
j =1;
for i = 1 : length(true_tree_location)
    if i<= size && j <= size
        
     if Trees_sensed(i,3) == 0
         j = j+1
     end
    dist = sqrt((Trees_sensed(j,1)-true_tree_location(i,1))^2 + (Trees_sensed(j,2)-true_tree_location(i,2))^2);
    dist_errors = [dist_errors dist];
    diam_errors = [diam_errors (true_tree_location(i,3)*2 - Trees_sensed(j,3))];
    
    j = j+1;
    end
     
    
end



% figure(5)
% histogram(diam_errors)
% title('Diameter Error')
% 
% figure(6)
% histogram(dist_errors)
% title('Position(dist.) Error')

% Distance error statistics 
Dist_mean = mean(dist_errors);
Dist_std = std(dist_errors);
Dist_error_max = max(dist_errors);
Dist_error_min = min(dist_errors);
Dist_error_RMS = rms(dist_errors);
Dist_error_95 = prctile(dist_errors,95)


% Diameter errors statistics
diam_errors = abs(diam_errors);
Diam_mean = mean(diam_errors);
Diam_std = std(diam_errors); 
Diam_error_max = max(diam_errors);
Diam_error_min = min(diam_errors);
Diam_error_RMS = rms(diam_errors);
Diam_error_95 = prctile(diam_errors,95)

disp("**Error Statistics for Dist Error**");
fprintf('mean = %f\n', Dist_mean); 
fprintf('Standard Deviation = %f\n', Dist_std);
fprintf('Max error = %f\n', Dist_error_max);
fprintf('Min error = %f\n', Dist_error_min);
fprintf('RMS = %f\n', Dist_error_RMS);
fprintf('95th pecentile = %f\n', Dist_error_95);

disp(" ");

disp("**Error Statistics for Diameter Error**");
fprintf('mean = %f\n', Diam_mean); 
fprintf('Standard Deviation = %f\n', Diam_std);
fprintf('Max error = %f\n', Diam_error_max);
fprintf('Min error = %f\n', Diam_error_min);
fprintf('RMS = %f\n', Diam_error_RMS);
fprintf('95th pecentile = %f\n', Diam_error_95);







 