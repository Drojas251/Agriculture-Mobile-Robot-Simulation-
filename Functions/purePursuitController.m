function [steer_angle, cross_track_error,goal,dmincor] = purePursuitController(q, L, ld, path);
% retuning the goal point and ploting it along with other elements in
% plottractor function
NumberOfPoints=size(path);
NumberOfPoints=NumberOfPoints(2);
T=transl2(q(1,1), q(2,1)) * trot2(q(3,1));
np=inv(T)*path;
s=0;
differentials=inf;
for i=1:NumberOfPoints
    Distances(i) = sqrt((q(1,1)-path(1,i)).^2 + (q(2,1)-path(2,i)).^2); % calculates all distances on the path from the robot
   

   if np(1,i)>0  %% all points which are closer than ld and are in front of the robot 
       
        s=s+1; 
        differentials(s)=abs(Distances(i) - ld);% all possible target ponts 
        target_point(s)=i; % index of target points

   end
 
end


if s>0 %% if there is at least one point within the range (closer than ld)
 %   goal=target_point(find(tp==max(tp))); % sets the goal point to the (index of) farthest point among the target ponts selected above
    goal_ind=find(differentials==min(differentials));
    nep=inv(T)*path(:,target_point(goal_ind)); % converts the goal point to the local coordinates
    goal=target_point(goal_ind);
%    plot(nep(1),nep(2),'og')
%    plot(path(1,goal),path(2,goal),'og') % uncomment to see the current goal point on the path
    ex=nep(1);
    ey=nep(2);
    dmin=min(Distances); % min distance of the robot from the path (Cross track Error) (unsigned)
    dmin_ind=find(Distances==dmin);
    nep=inv(T)*path(:,dmin_ind); % gets the corrdinates of the point which is used to calculate the cross track error in robot frame
%      plot(path(1,dmin_ind),path(2,dmin_ind),'go')
    dmincor=path(:,dmin_ind);
    cross_track_error=sign(nep(2))*dmin; % signed cross track error
    K=(2*ey)/ld^2;
    steer_angle=atan(K*L);
end
if s==0 ; steer_angle=-100; goal=0;cross_track_error=0;dmincor=path(:,1); end % when we reach the end of the path


end
