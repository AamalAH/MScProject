% produced by zijie lin, phd student from University of Maryland, College Park. copyright reserved.
function [Uthetav,mm,m]=multi_static_auto_ts(obstacle)
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
    Vu=15; Uz=600;
    Ts=15; dt=0.01; Tres=0.1;  % UAV heading time, time step and response time
    %---------------------------%
    Adistance0=Am(:,2);  % initial distance between uav and obstacles
    Athetas0=Am(:,3);    % initial distance angle
    Ax0=Adistance0.*cos(Athetas0); Ay0=Adistance0.*sin(Athetas0); 
    %---------------------------%
    %Az=[590;615;610];   % will change horizonal direction
    Az=Am(:,6);          % obstacle inital altitude
    Avelocity=Am(:,7);   % obstacle initial velocity       
    Athetav=Am(:,8);     % obstacle initial velocity angle
    Vax=Avelocity.*cos(Athetav); Vay=Avelocity.*sin(Athetav);
    aax=Am(:,11); aay=Am(:,12);
    

    Azmax=max(Az); Azmin=min(Az);  % maximum and minimum altitude of the obstacles
    deltamax_z=0.2; Uthetav=0;  % maximum clibing rate for uav in z and the initial velocity direction of uav
    
    Tc=((Ax0-2*R)-(dm-abs(Ay0)))/Vu-Tres;  %rp=Adistance*cos(theta_t0)-R;    % the critical starting time
    tc=min(Tc);
    if tc<0
       tc=0;
    end
    
    Ax=Ax0-Vu*tc; Ay=Ay0;  % calculating the critical starting location
    Athetas=atan(Ay./Ax);  % the angle of obstacle location after applying critical starting time
    for i=1:length(Athetas) % the angle in 2pi coordinates
        if Athetas(i)<=0
           Athetas(i)=Athetas(i)+2*pi;
        end
        Adistance(i,1)=sqrt(Ax(i)^2+Ay(i)^2);
    end
    An=[number Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];  % the update obstacle information to uav at critical time
    Ux(1)=0;Uy(1)=0;
    num=1; % the proportion to the Ts
    Kp=1;
    %------------------------------------------------------------------%
    if Uz>=Azmax  % if the height of uav is higher than any buildings
          disp('use heading up algorithm')
          deltamax_z=0.2;  %  test again
          delta=deltamax_z;  % maximum clibing angle is 0.2 rad.
          Uz1(1)=Uz; 
          
          %----------use PI control----------%
%           slope=Vu/R;
%           delta=deltamax_z;   % required pitching angle, input
%           num=1;         %num=num*Kp;  % number=0.6, den=[1 0.2]
%           den=[1 0.001]; %den=[0.5 0.08];num=0.8;
%           [numCL,denCL]=cloop(Kp*num,den, -1);
%           theta_sim=delta*step(numCL,denCL,0:dt:Ts);
%           
%           Kp9=10; Kd9=0;  Ki9=1;% Kd5=0.2;  %Kp5=0.8; Kd5=0.5; 0.375(s+2)/(s+0.25)
%           numc9=[Kd9 Kp9 Ki9];
%           denc9=[1 0];
%           [num9,den9]=cloop(conv(num,numc9),conv(den,denc9));
%           theta1=delta*step(num9,den9,0:dt:Ts);
          
        
          theta1(1)=0;i=0;
          for t=0:dt:num*Ts;
              i=i+1;
              thetadot(i)=Kp*(delta-theta1(i));
              theta1(i+1)=theta1(i)+thetadot(i)*dt;
          end  
          k=1;
          for t1=dt:dt:num*Ts
              k=k+1;
              heading(k)=theta1(k);            
              Uz1(k)=Uz1(k-1)+Vu*dt*sin(theta1(k));
              if abs(theta1(k))>=abs(delta)*0.99
                 break; 
              end              
          end
          Vuz=Vu*sin(theta1(k));
          for t2=t1+dt:dt:Ts 
              k=k+1;
              Uz1(k)=Uz1(k-1)+Vuz*dt;
              heading(k)=theta1(k);%%%%%%%%%%%j1?
              if Uz1(k)-Azmax>=dm
                 break;
              end               
          end
          
          %theta2=-delta*step(num9,den9,0:dt:Ts)+theta1(end);
          k3=1;
          theta2(1)=delta;i=0;
          for t=0:dt:num*Ts;
              i=i+1;
              theta2dot(i)=Kp*(0-theta2(i));
              theta2(i+1)=theta2(i)+theta2dot(i)*dt;
          end           
          for t3=t2+dt:dt:Ts
              k=k+1;
              k3=k3+1;
              heading(k)=theta2(k3);
              Uz1(k)=Uz1(k-1)+Vu*dt*sin(theta2(k3));
              if theta2(k3)<0.01
                 break; 
              end            
          end
          for t4=t3+dt:dt:Ts/dt 
              k=k+1;
              Uz1(k)=Uz1(k-1);
              heading(k)=0;
          end                    
          a=0;
          for t5=dt:dt:Ts         % the original xy trajectory of the uav
               a=a+1;
               Ux(a)=Vu*t5*cos(Uthetav);
               Uy(a)=Vu*t5*sin(Uthetav);
               rz1(:,a)=Uz1(a)-Az;
          end
          rz=(Uz-Az)*ones(1,a); 
          m=abs(Uz1(end)-Az);
          mm=Uz1(end)-Azmax;           % the update minimum vertical distance between uav and obstacles
%--------------------change direction in horizonal-----------------------%
     else    
          disp('change horizontal direction') 
          for i=1:length(Adistance)
               theta_d(i,1)=asin((dm)/Adistance(i)); % calculate the dangerous range 
          end

          for i=1:length(Adistance)  %%%% add control here
               range(i,1)=pi+atan(Ay(i)/Ax(i))-theta_d(i);  % transfer the dangerous range into 2pi coordinates
               range(i,2)=pi+atan(Ay(i)/Ax(i))+theta_d(i);
          end
    
          theta_vr0=pi*ones(length(Athetas),1);  % for uav, the velocity of the obstacles are pi.
    
    %----------this part is just for the test------------%
            b=0;
            for i=1:length(Adistance)
               if theta_vr0(i)>range(i,1)&& theta_vr0(i)<range(i,2)
                  b=b+1;
                  danger(b)=Am(1,1);
               end
            end
    %----------------------------------------------------%
           
           for i=1: length(Adistance)  % calculate for each obstacles, the direction variation needed for uav.
               alfa(i,1)=theta_d(i)-atan(Ay(i)/Ax(i));  %%%% add control here
               alfa(i,2)=theta_d(i)+atan(Ay(i)/Ax(i));
           end
                     
            delta1=max(abs(alfa(:,1))); % turning south
            delta2=max(abs(alfa(:,2))); % turning north
           if delta1<=delta2
              Uthetav=-delta1-0.05;   %%%% add control here
           else     
              Uthetav=delta2+0.05;
           end
           
           rx(:,1)=Ax(:,1)-Ux(1);
           ry(:,1)=Ay(:,1)-Uy(1); 
           for i=1:length(rx(:,1))    % part of new relative distance between obstacles and the uav
               r(i,1)=sqrt(rx(i,1)^2+ry(i,1)^2);
           end
    %-----------------------------------------------------------%
      % this part is to let uav use the new path and calculate the new distance to the obstacles
          k=1; 
%           num=1; Kp=1; %num=num*Kp;  % number=0.6, den=[1 0.2]
%           den=[1 0.001]; %den=[0.5 0.08];num=0.8;
%           [numCL,denCL]=cloop(Kp*num,den, -1);
%           
%           Kp9=10; Kd9=0;  Ki9=1;% Kd5=0.2;  %Kp5=0.8; Kd5=0.5; 0.375(s+2)/(s+0.25)
%           numc9=[Kd9 Kp9 Ki9];
%           denc9=[1 0];
%           [num9,den9]=cloop(conv(num,numc9),conv(den,denc9));
%           theta1=Uthetav*step(num9,den9,0:dt:Ts);
          
          theta1(1)=0;i=0;
          for t=0:dt:num*Ts;
              i=i+1;
              thetadot(i)=Kp*(Uthetav-theta1(i));
              theta1(i+1)=theta1(i)+thetadot(i)*dt;
          end  
          for t21=dt:dt:num*Ts
              k=k+1;
              heading(k)=theta1(k);
              Ux(k)=Ux(k-1)+Vu*dt*cos(theta1(k));
              Uy(k)=Uy(k-1)+Vu*dt*sin(theta1(k));
              
              Ax(:,k)=Ax(:,1)+Vax*t21;   % the trajectory of the obstacles under the coordinate applying critical time tc
              Ay(:,k)=Ay(:,1)+Vay*t21;
              rx(:,k)=Ax(:,k)-Ux(k);
              ry(:,k)=Ay(:,k)-Uy(k);            
              for i=1:length(rx(:,1))    % part of new relative distance between obstacles and the uav
                  r(i,k)=sqrt(rx(i,k)^2+ry(i,k)^2);
              end
                            
              if abs(theta1(k))>=abs(Uthetav)*0.99
                 break; 
              end             
          end
    Vux=Vu*cos(theta1(k));
    Vuy=Vu*sin(theta1(k));
    for t22=t21+dt:dt:num*Ts           % uav trajectories
    %for j2=j1+1:1:Ts/dt           % uav trajectories
        k=k+1;
        heading(k)=theta1(k);
        Ux(1,k)=Ux(1,k-1)+Vux*dt;
        Uy(1,k)=Uy(1,k-1)+Vuy*dt;
        Ux0(1,k)=Vu*t22;
        Uy0(1,k)=0;
        
        Ax(:,k)=Ax(:,1)+Vax*t22;    % trajectories of the obstacles 
        Ay(:,k)=Ay(:,1)+Vay*t22;
        rx(:,k)=Ax(:,k)-Ux(k);
        ry(:,k)=Ay(:,k)-Uy(k);
            
        for i=1:length(rx(:,1))     % the rest part of new relative distance between obstacles and the uav
            r(i,k)=sqrt(rx(i,k)^2+ry(i,k)^2);
        end
    end    
    m=min(r');                       
    mm=min(m);                      % minimum distance between the uav and the obstacles
    for l=1:length(Ax(:,1))
        if imag(alfa(l,1))==0 && imag(alfa(l,2))==0 
        else
           m(l)=dm-10;
           mm=dm-10;
        end
    end
   end
end