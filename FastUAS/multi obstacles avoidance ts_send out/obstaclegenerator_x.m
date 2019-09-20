% produced by zijie lin, phd student from University of Maryland, College Park. copyright reserved.
function [A,casenumber]=obstaclegenerator_x()
    global Vu_x0 Vu_y0 Vu_z0 Pu_x0 Pu_y0 Pu_z0 Va_x0 Va_y0 Pa_x0 Pa_y0 Pa_z0 aa_x aa_y d_m T 
    global Vu0 Vu theta_u0 theta_a0 t_u0 Va0 aa Pu_x Pu_y theta_u Pa_x Pa_y t_a0 d taw
    global dtheta_u_max dV_u_max Vu_max Vu_min 
    generatorR=1000; 
    searchR=500;  % radar search radius
    dangerR=500;
    d_m=30; %minimum allowable separation distance. can be smaller
    T=20; %look ahead time. 
    casenumber=8;  % buildings, no planes (currently we ignore the bird)
    %subcase=1;     % random generating
    subcase=2;    
    %casenumber=2; % one plane
    %casenumber=3; % one building, one plane
    %casenumber=4; % several planes
    %casenumber=5; % one building, several planes
    
    
%------------------------------------%
    if casenumber==1  % flight at low or mid air, randomly generator buildings(but probably one) as obstacle, no planes       
        if subcase==1
          n1=15;
          Adistance=generatorR*rand(n1,1);
          Athetas=2*pi*rand(n1,1);
          %Athetav=2*pi*rand(n1,1);
          Athetav=zeros(n1,1);
          Ax=zeros(n1,1);
          Ay=zeros(n1,1);
          Az=500+100*rand(n1,1);
          for n=1:n1
              Ax(n)=Adistance(n)*cos(Athetas(n));
              Ay(n)=Adistance(n)*sin(Athetas(n));
          end
          Avelocity=zeros(n1,1);
          Vax=zeros(n1,1);
          Vay=zeros(n1,1);
          aax=zeros(n1,1);
          aay=zeros(n1,1);
          %A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
          potentialdanger=0;
          for n=1:n1
              if Adistance(n)<dangerR
                 potentialdanger=potentialdanger+1;
              end
          end
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];  
    %-----------------------------------%
       elseif subcase==2
          Adistance=50+50*rand(1,1);
          Athetas=-pi/10+pi/5*rand(1,1);
          Athetav=0;
          Az=550+50*rand(1,1); %Az=1960;         
          %Athetas=0.7; % test for right turn of horizonal path change
          %Athetav=0.55;
          %Adistance=64.4;  % test for left turn of horizonal path change
          %Athetas=0.121;
          %Athetav=0.35;
          %Az=570;
          Ax=Adistance*cos(Athetas);
          Ay=Adistance*sin(Athetas);
          Avelocity=0;
          Vax=0;
          Vay=0;
          aax=0;
          aay=0;
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];
       else
          disp('wrong subcase input')
       end
    end
    
    if casenumber==2  % flight at high air, one aircraft as obstacle
       if subcase==1
          Adistance=dangerR*rand(1,1);
          Athetas=2*pi*rand(1,1);         
          Ax=Adistance*cos(Athetas);
          Ay=Adistance*sin(Athetas);
          Az=1950+100*rand(1,1); %Az=2045;
          Avelocity=120*rand(1,1);
          Athetav=2*pi;
          Vax=Avelocity*cos(Athetav);
          Vay=Avelocity*sin(Athetav);
          aax=0; aay=0;
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];
            %        if Athetas>=0 && Athetas<pi/2;
            %           subcase=1;
            %        end
            %        if Athetas>=pi/2 && Athetas<pi;
            %           subcase=2;
            %        end
            %        if Athetas>=pi && Athetas<1.5*pi;
            %           subcase=3;
            %        end
            %        if Athetas>=1.5*pi && Athetas<2*pi;
            %           subcase=4;
            %        end
       elseif subcase==2
          Adistance=90;
          Athetas=0.2;
          Ax=Adistance*cos(Athetas);
          Ay=Adistance*sin(Athetas);
          Az=1970+60*rand(1,1); %Az=2045;
          Avelocity=50; %Avelocity=50+50*rand(1,1); 
          Athetav=5.0;
          Vax=Avelocity*cos(Athetav);
          Vay=Avelocity*sin(Athetav);
          aax=0; aay=0;
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
       end
    end
    
    if casenumber==3   % flight at mid air, one building, one plane as obstacles
       %Adistance(1,1)=50+50*rand(1,1);
       %Adistance(2,1)=dangerR/5*rand(1,1);
       %Athetas=2*pi*rand(n3,1);
       %Athetav=2*pi*rand(n3,1);
       Adistance=[90;150];
       Athetas=[0.2;6.0];
       Ax=Adistance.*cos(Athetas);
       Ay=Adistance.*sin(Athetas);
       Az=[580;629];
       Avelocity=[0;60];
       Athetav=[0;0.85*pi];
       Vax=Avelocity.*cos(Athetav);
       Vay=Avelocity.*sin(Athetav);
       aax=[0;0]; aay=[0;0];
       A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];  
    end
       
    if casenumber==4  % flight at high air, 2-3 planes as obstacles
       if subcase==1  % randomly generate
           n4=20;         %%% rewrite!
           Adistance=dangerR/2*rand(n4,1);
           Athetas=2*pi*rand(n4,1);
           Ax=Adistance.*cos(Athetas);
           Ay=Adistance.*sin(Athetas);
           Az=1950+100*rand(n4,1);
           Avelocity=120*rand(n4,1);
           Athetav=2*pi*rand(n4,1);    
           Vax=Avelocity.*cos(Athetav);
           Vay=Avelocity.*sin(Athetav);
           aax=zeros(n4,1);
           aay=zeros(n4,1);    
       elseif subcase==2   % two obstacles from behind
           Adistance=[80;100];
           Athetas=[0.75*pi; 4/3*pi];
           Ax=Adistance.*cos(Athetas); 
           Ay=Adistance.*sin(Athetas); 
           Az=[1975;2025];
           Avelocity=[110;120]; 
           Athetav=[1.95*pi;pi/8];
           Vax=Avelocity.*cos(Athetav); 
           Vay=Avelocity.*sin(Athetav);
           aax=[0;0]; aay=[0;0];         
       elseif subcase==3  % two obstacles from front
           Adistance=[120;150];
           Athetas=[0.5; 5.6];
           Ax=Adistance.*cos(Athetas); 
           Ay=Adistance.*sin(Athetas); 
           Az=[1985;2015];  
           Avelocity=[60;70]; 
           Athetav=[1.25*pi;0.6*pi];
           Vax=Avelocity.*cos(Athetav); 
           Vay=Avelocity.*sin(Athetav);
           aax=[0;0]; aay=[0;0];
       else                     % four obstacles from behind and front
           Adistance=[80;100;120;150];
           Athetas=[0.75*pi;4/3*pi;0.5;5.6];
           Ax=Adistance.*cos(Athetas); 
           Ay=Adistance.*sin(Athetas); 
           Az=[1985;1995;2005;2015];
           Avelocity=[110;120;60;70]; 
           Athetav=[1.95*pi;pi/8;1.25*pi;0.6*pi];
           Vax=Avelocity.*cos(Athetav); 
           Vay=Avelocity.*sin(Athetav);
           aax=[0;0;0;0]; aay=[0;0;0;0];
       end
       A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; % the data of the obstacles.
    end
    
    if casenumber==5  % flight at mid air, there is one building, one aircraft and one bird
%        n5=10;
%        Adistance(1,1)=50+50*rand(1,1);
%        Adistance(2:n5,1)=300*rand(n5-1,1);
%        Az(1,1)=600+100*rand(1,1);
%        Az(2:n5,1)=2000-100*rand(n5-1,1);
%        Athetas=2*pi*rand(n5,1);
%        Athetav=2*pi*rand(n5,1);
%        Ax=Adistance.*cos(Athetas);
%        Ay=Adistance.*sin(Athetas);
%        
%        Avelocity(1,1)=0;
%        Avelocity(2:n5,1)=40*rand(n5-1,1)-20;
%        Vax=Avelocity.*cos(Athetav);
%        Vay=Avelocity.*cos(Athetav);
%        aax(1,1)=0;
%        aax(2:n5,1)=-5+15*rand(n5-1,1);
%        aay(1,1)=0;
%        aay(2:n5,1)=-5+15*rand(n5-1,1);  
%        A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; % the data of the obstacles.
       Adistance=[90;150;50];
       Athetas=[0.2;6.0;0.5];
       Ax=Adistance.*cos(Athetas);
       Ay=Adistance.*sin(Athetas);
       Az=[580;629;600];
       Avelocity=[0;60;10];
       Athetav=[0;0.85*pi;1.45*pi];
       Vax=Avelocity.*cos(Athetav);
       Vay=Avelocity.*sin(Athetav);
       aax=[0;0;-1]; aay=[0;0;-1];
       A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];
    end   
    %-----------------------complicated situations-----------------------%
    if casenumber==6  % flight at low or mid air, randomly generator buildings(but probably one) as obstacles, no planes       
        if subcase==1   % randomly generate 20 ojects, to see which ones are the real obstacles for uav
          n1=20;
          %Adistance=generatorR*rand(n1,1);
          Adistance=300+dangerR/5*rand(n1,1);
          Athetas=2*pi*rand(n1,1);
          Ax=Adistance.*cos(Athetas);
          Ay=Adistance.*sin(Athetas);
          Az=550+100*rand(n1,1);
          Avelocity=zeros(n1,1);
          Athetav=zeros(n1,1);
          Vax=zeros(n1,1);
          Vay=zeros(n1,1);
          aax=zeros(n1,1);
          aay=zeros(n1,1);
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];  
    %-----------------------------------%
       elseif subcase==2   % randomly generate 3 real obstacles
          n1=6;
          Adistance=50+dangerR/2*rand(n1,1);
          Athetas=-pi/10+pi/5*rand(n1,1);
          
          Az=570+60*rand(n1,1); %Az=1960;         
          %Athetas=0.7; % test for right turn of horizonal path change
          %Athetav=0.55;
          %Adistance=64.4;  % test for left turn of horizonal path change
          %Athetas=0.121;
          %Athetav=0.35;
          %Az=570;
          Ax=Adistance.*cos(Athetas);
          Ay=Adistance.*sin(Athetas);
          Avelocity=zeros(n1,1);
          Athetav=zeros(n1,1);
          Vax=zeros(n1,1);
          Vay=zeros(n1,1);
          aax=zeros(n1,1);
          aay=zeros(n1,1);
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];
        else                             % subcase==3;
          n1=6;                          % generate 3 certain obstacles
          Adistance=[54; 63; 80; 60; 120; 160];
          %Athetas=[-0.1865; -0.2684; -0.1962;  0.0375;  0;  -0.05];
          Athetas=[0.1865; 0.2684; 0.1962;  0.0375;  0;  0.05];
          Ax=Adistance.*cos(Athetas);
          Ay=Adistance.*sin(Athetas);
          Az=[608;  595;  593;  618;  589;  615];
          Avelocity=zeros(n1,1);
          Athetav=zeros(n1,1);
          Vax=zeros(n1,1);
          Vay=zeros(n1,1);
          aax=zeros(n1,1);
          aay=zeros(n1,1);
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
       end
    end
    if casenumber==7   % flight at mid air, several buildings and planes
       if subcase==1   % note that subcase=1 means the random generation, subcase=2 means being fixed some level.
           n1=20; n2=[1/3*n1]; % n2 is the number of the static obstacle
           Adistance=searchR*rand(n1,1);
           Az=500+200*rand(n1,1);
           Athetav(n2+1:n1)=2*pi*rand(n1-n2,1);
           Ax=Adistance.*cos(Athetas);
           Ay=Adistance.*sin(Athetas);
       
           Avelocity=zeros(n1,1);
           Avelocity(n2+1:n1,1)=150*rand(n1-n2,1);
           Athetas=2*pi*rand(n1,1);
           Athetav=zeros(n1,1);
           Vax=Avelocity.*cos(Athetav);
           Vay=Avelocity.*sin(Athetav);
           aax=zeros(n1,1);
           aay=zeros(n1,1);
           A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
       end
       if subcase==2    
           n1=6; n2=2; % n2 is the number of the static obstacle
           Adistance=dangerR/5*rand(n1,1);
           Az=570+60*rand(n1,1);
           Athetas=-pi/6+pi/3*rand(n1,1);
           Ax=Adistance.*cos(Athetas);
           Ay=Adistance.*sin(Athetas);
       
           Avelocity=zeros(n1,1);
           Avelocity(n2+1:n1,1)=150*rand(n1-n2,1);
           Athetav=zeros(n1,1);
           Athetav(n2+1:n1)=[pi/10;2/3*pi;4/3*pi;1.85*pi;];
           Vax=Avelocity.*cos(Athetav);
           Vay=Avelocity.*sin(Athetav);
           aax=zeros(n1,1);
           aay=zeros(n1,1);
           A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
       end
    end
    if casenumber==8  % flight at low or mid air, randomly generated planes as obstacle, no buildings       
        if subcase==1 % randomly generate 20 ojects, to see which ones are the real obstacles for uav
          n1=6;
          Adistance=300+dangerR/5*rand(n1,1);
          Athetas=2*pi*rand(n1,1);
          Ax=zeros(n1,1);
          Ay=zeros(n1,1);
          Az=500+200*rand(n1,1);
          for n=1:n1
              Ax(n)=Adistance(n)*cos(Athetas(n));
              Ay(n)=Adistance(n)*sin(Athetas(n));
          end
          Avelocity=20+5*rand(n1,1);  
          Athetav=2*pi*rand(n1,1);
          Vax=Avelocity.*cos(Athetav);
          Vay=Avelocity.*sin(Athetav);
          aax=zeros(n1,1);
          aay=zeros(n1,1);
          %A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay]; 
          potentialdanger=0;
          for n=1:n1
              if Adistance(n)<dangerR
                 potentialdanger=potentialdanger+1;
              end
          end
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];  
    %-----------------------------------%
       elseif subcase==2  % randomly generate 10 objects in the front and near the height to the uav.
          n1=10;
          Adistance=100+dangerR/2*rand(n1,1);         
          Athetas=-pi/2+pi*rand(n1,1);         
          Athetav=2*pi*rand(n1,1);         
          Az=570+60*rand(n1,1); %Az=1960;                  
          Ax=Adistance.*cos(Athetas);
          Ay=Adistance.*sin(Athetas);
          Avelocity=70*rand(n1,1);         
          Vax=Avelocity.*cos(Athetav);
          Vay=Avelocity.*sin(Athetav);
          aax=zeros(n1,1);
          aay=zeros(n1,1);
          for i=1:length(Athetas)  % just for thest the obstacle from ahead
              if Athetas(i)<0
                 Athetas(i)=Athetas(i)+2*pi;
              end
          end
          A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];
       else
          %Adistance(3:5)=[80;150;100];
          %Athetas=[0;pi/6;0.6*pi;pi;1.45*pi;1.85*pi];
          %Athetav=[pi;1.25*pi;1.75*pi;0;0.3*pi;0.7*pi]; %%%note 
          %Athetas=0.7; % test for right turn of horizonal path change
          %Athetav=0.55;
          %Adistance=64.4;  % test for left turn of horizonal path change
          %Athetas=0.121;
          %Athetav=0.35;
          %Az=570;
          %Avelocity(3:5)=[125;150;120];
          disp('wrong subcase input')
       end
    end
    if casenumber==9  % encountering the birds
       A=zeros(1,11);
       A(10)=-1; A(11)=-1; % -1 means we encoutering the birds, use sonar to drive away the birds.
    end
end