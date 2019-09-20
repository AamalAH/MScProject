% produced by zijie lin, phd student from University of Maryland, College Park. copyright reserved.
function [a,Obstacle]=detection(A,casenumber)
   global Vu_x0 Vu_y0 Vu_z0 Pu_x0 Pu_y0 Pu_z0 Va_x0 Va_y0 Pa_x0 Pa_y0 Pa_z0 aa_x aa_y dm T A
   global Vu0 Vu theta_u0 theta_a0 t_u0 Va0 aa Pu_x Pu_y theta_u Pa_x Pa_y t_a0 d taw
   global dtheta_u_max dV_u_max Vu_max Vu_min 

   %A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];
   Adistance=A(:,1); Athetas=A(:,2); Ax=A(:,3); Ay=A(:,4); Az=A(:,5);
   Avelocity=A(:,6); Athetav=A(:,7); Vax=A(:,8); Vay=A(:,9); aax=A(:,10); aay=A(:,11);
   
   Vu=15; 
   %Vux=50; Vuy=0; Vuz=0; 
   Uz=600; % Vu is the obstacle speed, Vux, Vuy, Vuz are the xyz component of the Vu.
   dm=30; dmz=30; % dm/dmz is the horizotal and vertical safe separation distance of the uav
   
   Vrx0(:,1)=Vax(:,1)-Vu;  % in original path, Vux=Vu, Vuy=0;
   Vry0(:,1)=Vay(:,1);
   theta_vr0=atan(abs(Vry0./Vrx0));  % theta_vr0 is the angle of relative distance between uav and the obstacles
   %--------------transfer theta_vr0 in the 2 pi range---------------%
   for i=1:length(theta_vr0)
       if Vrx0(i)==0
          if Vry0(i)>0
             theta_vr0(i)=pi/2;
          elseif vry0(i)<0
             theta_vr0(i)=1.5*pi;
          else
             theta_vr0=100;  % it means that both relative x and y velocity is 0, thus they will not encounter
          end
       else      
          if Vrx0(i)<0 && (Vry0(i)<0 || Vry0(i)==0)
              theta_vr0(i)=theta_vr0(i)+pi;
          elseif Vrx0(i)<0 && Vry0(i)>0
              theta_vr0(i)=pi-theta_vr0(i); 
          elseif Vrx0(i)>0 && Vry0(i)<0
              theta_vr0(i)=2*pi-theta_vr0(i);  % Vrx0>0; Vry0>0; theta_vr0 doesn't need to change
          end
        end       
   end
   %------------------------------------------%
   
   Athetas_re=Athetas; % for later computation, we need the angle that start from the obstacles.
   for i=1:length(A(:,1))
       if Athetas(i)<0
          Athetas_re(i)=Athetas(i)+pi;
       elseif Athetas(i)<pi
          Athetas_re(i)=Athetas(i)+pi;
       else
          Athetas_re(i)=Athetas(i)-pi;
       end
   end
   dangle=abs(theta_vr0-Athetas_re);    % calculate the angle between the relative distance and relative velocity
   for i=1:length(Adistance)
        dc(i)=Adistance(i)*sin(dangle(i));
        dcz(i)=abs(Az(i)-Uz);
   end
   a=0;
   for i=1:length(Adistance)
       if A(i,10)==-1
          a=a+1;
          Obstacle(a,:)=A(i,:);
          disp('warning, a school of birds!') 
       elseif (dangle(i)<pi/2) && (dc(i)<dm) && (dcz(i)<dmz)  % satisfy the requirements, then the item is the real obstacle for uav
           a=a+1;
           Obstacle(a,:)=A(i,:);
           if Obstacle(a,6)==0
              disp('warning, static obstacle found!')
           else
              disp('warning, dynamic obstacle found!')
           end
           disp('distance   direction   velocity   velocity direction')
           disp([Obstacle(a,1), Obstacle(a,2), Obstacle(a,6), Obstacle(a,7)])
       end
   end
   
   if a==0   % if no obstacles, then continue flying
      Obstacle=zeros(1,11);
      disp('safe, continue flight!')
   end
   
   for i=1:length(Adistance)
        theta_d(i,1)=asin((dm)/Adistance(i));
   end
%-----------------------------------------------------------------------%
   
%    if  length(Obstacle(:,1))==1
%         for i=1:length(Obstacle)  % if obstacle has several rows, there must be an obstacle. But if obstacle only has one row, might it is an null row, thus there is only one loop i, not the loop i,j.
%             if Obstacle(i)==0
%                a=a+1;
%             end
%         end
%         if a==length(Obstacle(1,:))
%         else
%            avoidance(Obstacle)
%         end
%     else
%         avoidance(Obstacle)
%     end
   