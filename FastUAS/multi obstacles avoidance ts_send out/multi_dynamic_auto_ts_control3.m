% produced by zijie lin, phd student from University of Maryland, College Park. copyright reserved.
function [Uthetav,mm,m]=multi_dynamic_auto_ts_control3(obstacle)
    %Am=[number Adistance0 Athetas0 Ax0 Ay0 Az Avelocity Athetav Vax Vay aax aay];
    % Am saves the obstacles information
    for i=1:length(obstacle(:,1))
        number(i,1)=i;
    end
    Am=[number,obstacle];
    start_point=[-200,0,0]; goal_point=[10000,0,0];
    foot_points=[0,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000];
    
    theta_u=0;   % the angle of the uav velocity, assume uav flying east.
    dm=32; R=40; % uav separation radius and turn rate
    Vu=15; Uz=600;   %%% 7.25 revise
    Ts=15; dt=0.01; Tres=0.1;  % UAV heading time, time step and response time
    
    %----------------------------------%
    Adistance0=Am(:,2);   % initial distance between uav and obstacles
    Athetas0=Am(:,3);     % initial distance angle
    Ax0=Adistance0.*cos(Athetas0); Ay0=Adistance0.*sin(Athetas0); 
    %----------------------------------%    
    Az=Am(:,6);           % obstacle inital altitude
    Avelocity=Am(:,7);    % obstacle initial velocity
    Va=Avelocity;
    Athetav=Am(:,8);      % obstacle initial velocity angle
    Vax=Avelocity.*cos(Athetav); Vay=Avelocity.*sin(Athetav);  % obstacles velocity in xy direction
    aax=Am(:,11); aay=Am(:,12);

    Azmax=max(Az); Azmin=min(Az);     % maximum and minimum altitude of the obstacles
    r0=Adistance0; rxo=Ax0; ryo=Ay0;
    deltamax_z=0.2; Uthetav=0;        % maximum clibing rate for uav in z and the initial velocity direction of uav
    
    Vrx0(:,1)=Vax(:,1)-Vu;  % in original path, Vux=Vu, Vuy=0;
    Vry0(:,1)=Vay(:,1);     % relative velocity
    Vr0=sqrt(Vrx0.^2+Vry0.^2);
    
    theta_r0=atan(abs(Ay0./Ax0));  % angle for relative distance
    theta_vr0=atan(abs(Vry0./Vrx0));  % angle for relative velocity
    theta_t0=abs(theta_r0-theta_vr0); % angle difference between relative distance angle and relative velocity angle
    
    Tc=(r0.*cos(theta_t0)+r0.*sin(theta_t0)-R-2*dm)./Vr0-Tres;  %rp=Adistance*cos(theta_t0)-R;  % the critical starting time
    tc=min(Tc);
    
    Ax=Ax0-Vu*tc; Ay=Ay0;  % calculating the critical starting location
    Athetas=atan(Ay./Ax);  % the angle of obstacle location after applying critical starting time
    for i=1:length(Athetas)  % the angle in 2pi coordinates
        if Athetas(i)<=0
           Athetas(i)=Athetas(i)+2*pi;
        end
        Adistance(i,1)=sqrt(Ax(i)^2+Ay(i)^2);
    end
    An=[number Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];  % the update obstacle information to uav at critical time
    
    %------------------------------------------------------------------%
    if Uz>=Azmax|| Uz<=Azmin  % if the height of uav is higher or lower than any dynamic obstacles
       if Uz>=Azmax
          disp('heading up')
          deltamax_z=0.2;  
          delta=deltamax_z;  % maximum clibing angle is 0.2 rad.
 
          k=1;
          Uz1(1)=Uz; 
          Kp=2;
          %-------------control on the theta--------------%
          theta1(1)=0;i=0;
          for t=0:dt:Ts;
              i=i+1;
              thetadot(i)=Kp*(delta-theta1(i));
              theta1(i+1)=theta1(i)+thetadot(i)*dt;
          end         

          for t1=dt:dt:Ts   % uav start to turning up
                 k=k+1;
                 heading(k)=theta1(k);                 
                 Uz1(k)=Uz1(k-1)+Vu*dt*sin(theta1(k));  %Uz1(k)=Uz+R-R*cos(theta1(k));
                 if abs(theta1(k))>=abs(delta)
                    break;
                 end
          end
          Vuz=Vu*sin(theta1(k));
          for t2=t1+dt:dt:Ts       % uav heading up to higher altitude 
                 k=k+1;
                 Uz1(k)=Uz1(k-1)+Vuz*dt;
                 heading(k)=heading(k-1);
                 if abs(Uz1(k)-Azmax)>=35
                    break;
                 end 
          end
          
          k3=1;
          theta2(1)=delta;i=0;
          for t=0:dt:Ts;
              i=i+1;
              theta2dot(i)=Kp*(0-theta2(i));
              theta2(i+1)=theta2(i)+theta2dot(i)*dt;
          end      
          
          for t3=t2+dt:dt:Ts        % the clibing angle turn back to zero again   
                 k=k+1;
                 k3=k3+1;                 
                 heading(k)=theta2(k3); %heading(k)=theta1(end)-theta2(k3);        
                 Uz1(k)=Uz1(k-1)+Vu*dt*sin(heading(k));
                 if abs(theta2(k3))<0.01; %abs(theta2(k3))>abs(theta1(end))
                    break;
                 end            
          end       
          for t4=t3+dt:dt:Ts        % uav flying at higher altitude
                 k=k+1;
                 Uz1(k)=Uz1(k-1);
                 heading(k)=0;
          end 
          
          m=abs(Uz1(end)-Az);   
          mm=Uz1(end)-Azmax;           % the update minimum vertical distance between uav and obstacles
       
       else
           
          disp('fly lower')
          deltamax_z=0.2;  
          delta=-deltamax_z;
          k=1;
          Uz1(1)=Uz; 
          Kp=2;
          
          theta1(1)=0;i=0;
          for t=0:dt:Ts;
              i=i+1;
              thetadot(i)=Kp*(delta-theta1(i));
              theta1(i+1)=theta1(i)+thetadot(i)*dt;
          end  
          
          for t1=dt:dt:Ts/10        % uav start to turning down
                 k=k+1;                
                 Uz1(k)=Uz1(k-1)+Vu*dt*sin(theta1(k));
                 heading(k)=theta1(k);
                 if abs(theta1(k))>=abs(delta)
                    break;
                 end
          end
          Vuz=Vu*sin(theta1(k));
          for t2=t1+dt:dt:Ts         % uav heading up to lower altitude
                 k=k+1;                 
                 Uz1(k)=Uz1(k-1)+Vuz*dt;
                 heading(k)=heading(k-1);
                 if abs(Uz1(k)-Azmin)>=35                
                    break;
                 end 
          end
          
          k3=1;
          theta2(1)=delta;i=0;
          for t=0:dt:Ts;
              i=i+1;
              theta2dot(i)=Kp*(0-theta2(i));
              theta2(i+1)=theta2(i)+theta2dot(i)*dt;
          end   
          
          for t3=t2+dt:dt:Ts          % the clibing angle turn back to zero again   
                 k=k+1;
                 k3=k3+1;                
                 heading(k)=theta2(k3);
                 Uz1(k)=Uz1(k-1)+Vu*dt*sin(heading(k));
                 if abs(theta2(k3))<0.01  %abs(theta2(k3))>abs(theta1(end))
                    break;
                 end            
          end       
          for t4=t3+dt:dt:Ts          % uav flying at lower altitude
                 k=k+1;
                 Uz1(k)=Uz1(k-1);
                 heading(k)=0;
          end   
       end
       
       a=0;
       for t5=0:dt:Ts                  % the xy trajectory of the uav
           a=a+1;
           Ux(a)=Vu*t5*cos(Uthetav);
           Uy(a)=Vu*t5*sin(Uthetav);
       end
       m=abs(Uz1(end)-Az);
       mm=abs(Uz1(end)-Azmin);         % the update minimum vertical distance between uav and obstacles
       
%--------------------change direction in horizonal-----------------------%
   
    else   %this part is used to calculate the direction variation needed for uav.
      
       for i=1:length(Adistance)
           theta_d(i,1)=asin((dm)/Adistance(i));  % calculate the dangerous range
       end
    
    for i=1:length(Adistance)
         if Athetas(i)<3/2*pi && Athetas(i)>=pi/2
            if Athetas(i)<pi && Athetas(i)>=pi/2
               range(i,1)=2*pi-abs(atan(abs(Ay(i)/Ax(i)))-theta_d(i));  % transfer the dangerous range into 2pi coordinates
               range(i,2)=2*pi-abs(atan(abs(Ay(i)/Ax(1)))+theta_d(i));
            end
            if Athetas(i)<pi*3/2 && Athetas(i)>=pi
               range(i,1)=abs(atan(abs(Ay(i)/Ax(i)))+theta_d(i));
               range(i,2)=abs(atan(abs(Ay(i)/Ax(i)))-theta_d(i));
            end
        else
           range(i,1)=pi+atan(Ay(i)/Ax(i))-theta_d(i);
           range(i,2)=pi+atan(Ay(i)/Ax(i))+theta_d(i);
        end
    end
    %----------------------------------------------------%
           
           for i=1: length(Adistance) % followings are geometry method
               if Athetas(i)<3/2*pi && Athetas(i)>=pi/2
               else                  
                   alfa(i,1)=theta_d(i)-atan(Ay(i)/Ax(i));
                   alfa(i,2)=theta_d(i)+atan(Ay(i)/Ax(i));
               end
           end
           
           belta=atan(abs(Vay./Vax));

           %theta=abs(belta-abs(alfa));% check here
           theta(:,1)=abs(belta-abs(alfa(:,1)));% check here
           theta(:,2)=abs(belta-abs(alfa(:,2)));% check here


       for i=1:length(Adistance)           
               if Va(i)*cos(theta(i,1))>=sqrt(Vu^2-Va(i)^2*sin(theta(i,1))^2)
                    l(i,1)=Va(i)*cos(theta(i,1))-sqrt(Vu^2-Va(i)^2*sin(theta(i,1))^2);
               else
                    l(i,1)=Va(i)*cos(theta(i,1))+sqrt(Vu^2-Va(i)^2*sin(theta(i,1))^2);
               end
           
               if Va(i)*cos(theta(i,2))>=sqrt(Vu^2-Va(i)^2*sin(theta(i,2))^2)
                  l(i,2)=Va(i)*cos(theta(i,2))-sqrt(Vu^2-Va(i)^2*sin(theta(i,2))^2);
               else
                  l(i,2)=Va(i)*cos(theta(i,2))+sqrt(Vu^2-Va(i)^2*sin(theta(i,2))^2);
               end
               if Vu^2-Va(i)^2*sin(theta(i,1))^2>=0 && Vu^2-Va(i)^2*sin(theta(i,2))^2>=0
                  check(i,1)=0; % continue calculation
               else
                  check(i,1)=1; % failure
               end
       end
       
       for i=1:length(Adistance)
           gama(i,1)=acos((l(i,1)^2+Vu^2-Va(i)^2)/(2*l(i,1)*Vu));
           gama(i,2)=acos((l(i,2)^2+Vu^2-Va(i)^2)/(2*l(i,2)*Vu));
       end
           
           lamda=abs(alfa-gama);
           
           delta1=max(lamda(:,1));    % turning south
           delta2=max(lamda(:,2));    % turning north
           if delta1<=delta2
              Uthetav=-delta1-0.05;
           else     
              Uthetav=delta2+0.05;
           end 
    %-----------------------test if successful---------------------------%
    % this part is to let uav use the new path and calculate the new distance to the obstacles   
    
    k1=1; Ux(1)=0; Uy(1)=0; Kp=2;
    
    theta1(1)=0;i=0;
    for t=0:dt:Ts;
        i=i+1;
        thetadot(i)=Kp*(Uthetav-theta1(i));
        theta1(i+1)=theta1(i)+thetadot(i)*dt;
    end
  
    rx(:,1)=Ax(:,1);
    ry(:,1)=Ay(:,1);            
    rx0(:,1)=rxo;
    ry0(:,1)=ryo;
    
    for i=1:length(rx(:,1))     % part of new relative distance between obstacles and the uav
        r(i,1)=sqrt(rx(i,1)^2+ry(i,1)^2);
        r0(i,1)=sqrt(rx0(i,1)^2+ry0(i,1)^2);
    end
    
    for t21=dt:dt:Ts

        k1=k1+1;       
        heading(k1)=theta1(k1);  % heading angle 
        %Ux(k)=R*sin(theta1(k1));
        %Uy(k)=-R+R*cos(theta1(k1));
        Ux(k1)=Ux(k1-1)+Vu*dt*cos(theta1(k1));
        Uy(k1)=Uy(k1-1)+Vu*dt*2*sin(theta1(k1)/2);        
                                     
        Ax(:,k1)=Ax(:,1)+Vax*t21;        % the trajectory of the obstacles under the coordinate applying critical time tc
        Ay(:,k1)=Ay(:,1)+Vay*t21;
        
        rx(:,k1)=Ax(:,k1)-Ux(k1);
        ry(:,k1)=Ay(:,k1)-Uy(k1);            
        rx0(:,k1)=rxo+Vrx0*t21;
        ry0(:,k1)=ryo+Vry0*t21;
        
        for i=1:length(rx(:,1))     % part of new relative distance between obstacles and the uav
            r(i,k1)=sqrt(rx(i,k1)^2+ry(i,k1)^2);
            r0(i,k1)=sqrt(rx0(i,k1)^2+ry0(i,k1)^2);
        end
        if abs(theta1(k1))>=abs(Uthetav)  % continue flying south/north
           break;
        end
    end
    
    Vux=Vu*cos(theta1(k1));        % xy velocity of the uav, variation to south and north  
    Vuy=Vu*sin(theta1(k1));
        
    for t22=t21+dt:dt:Ts            % uav trajectories
        k1=k1+1;
        heading(k1)=Uthetav;
        Ux(1,k1)=Ux(1,k1-1)+Vux*dt;
        Uy(1,k1)=Uy(1,k1-1)+Vuy*dt;
        Ux0(1,k1)=Vu*t22;
        Uy0(1,k1)=0;
        
        Ax(:,k1)=Ax(:,1)+Vax*t22;        % trajectories of the obstacles
        Ay(:,k1)=Ay(:,1)+Vay*t22;
        rx(:,k1)=Ax(:,k1)-Ux(k1);
        ry(:,k1)=Ay(:,k1)-Uy(k1);
            
        rx0(:,k1)=rxo+Vrx0*t22;
        ry0(:,k1)=ryo+Vry0*t22;
        for i=1:length(rx(:,1))
            r(i,k1)=sqrt(rx(i,k1)^2+ry(i,k1)^2);     % the rest part of new relative distance between obstacles and the uav
            r0(i,k1)=sqrt(rx0(i,k1)^2+ry0(i,k1)^2);
        end
    end
    
    m=min(r');                       % minimum distance between the uav and the obstacles
    mm=min(m);
    
    for i=1:length(check(:,1))
        if check(i,1)==1;  % check(i)=1 means that reactive value is generated so the avoidance case is fail.
           mm=dm-10;
           m(i)=dm-10;
        end
    end
    end
    %--------------------------------------------------------%
        
end