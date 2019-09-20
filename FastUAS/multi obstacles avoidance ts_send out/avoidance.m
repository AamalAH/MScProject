% produced by zijie lin, phd student from University of Maryland, College Park. copyright reserved.
function [Uthetav,mm,m]=avoidance(obstacle)
%A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
a=0; Ts=10; dt=0.01;            % a is to account the dynamic obstacles number among the total obstacles. Ts is the heading time, dt is the time step.
for i=1:length(obstacle(:,1))   % determine dynamic obstacles or static obstacles
    if obstacle(i,6)~=0
       a=a+1;
    end
end
Ax0=obstacle(:,3);  Ay0=obstacle(:,4);  Az0=obstacle(:,5);
Vax=obstacle(:,8);  Vay=obstacle(:,9);
if a==0       % all of them are static obstacles
   Ax=Ax0; Ay=Ay0; Az=Az0;  %  get the static obstacles locations
%    figure
%    plot(Ax,Ay,'.b');
%    xlim([0,200]);
else
   i=0;                     %  get the dynamic obstacles trajectories
   for t=0:dt:Ts
       i=i+1;
       Ax(:,i)=Ax0+Vax*t;
       Ay(:,i)=Ay0+Vay*t;
   end
%    figure
%    for j=1:length(obstacle(:,1))
%        hold on
%        plot(Ax(j,:),Ay(j,:),'-.b');
%    end
end
if a==0
   [Uthetav,mm,m]=multi_static_auto_ts(obstacle);  %avoid_static(obstacle);   
else
   [Uthetav,mm,m]=multi_dynamic_auto_ts_control3(obstacle); %avoid_dynamic(obstacle);   
end
