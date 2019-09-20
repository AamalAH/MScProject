% produced by zijie lin, phd student from University of Maryland, College Park. copyright reserved.
    clear;
    clc;
    global Vu_x0 Vu_y0 Vu_z0 Pu_x0 Pu_y0 Pu_z0 Va_x0 Va_y0 Pa_x0 Pa_y0 Pa_z0 aa_x aa_y d_m T A
    global Vu0 Vu theta_u0 theta_a0 t_u0 Va0 aa Pu_x Pu_y theta_u Pa_x Pa_y t_a0 d taw    
    global dtheta_u_max dV_u_max Vu_max Vu_min failure_matrix f_num f
%--------------------input the value of parameters-----------------------%
%-----------------------obstacles generator---------------------%
%A=[Adistance Athetas Ax Ay Az Avelocity Athetav Vax Vay aax aay];

n=7500;dm=30; h=0.95; % n: number of the cases. dm: the uav safe radius h:the percentage of the dm
a=zeros(n,1); Uthetav=zeros(n,1); mm=zeros(n,1); M1=zeros(n,20); % a: accounts the obstacle number in each case
f_num=0;f=0; failure_matrix=zeros(n/5,26);
%---------------finish the marte carlo simulation first------------------%
TIMES = [];

for k=1:n
    tic
    [A,casenumber]=obstaclegenerator_x(); 
    [a(k,1),obstacle]=detection(A,casenumber);
    if a(k,1)~=0  % there has obstacles
       [Uthetav(k),mm(k),m]=avoidance(obstacle); % mm is the minimum relative dist. after uav change its way.
       %[Uthetav(k)]=avoidance(obstacle);
    else
       Uthetav(k)=0;mm(k)=35; % if no obstacles, uav continue its path.
    end
    %--------------------%
    if mm(k)<30;
       f_num=f_num+1;
       for i=1:length(obstacle(:,1));
           f=f+1;
           failure_matrix(f,1)=f_num;
           if m(i)<30;               
              failure_matrix(f,2)=1;              
           else
              failure_matrix(f,2)=0;
           end
           failure_matrix(f,3:13)=obstacle(i,:); 
       end
    else       
    end
    %-----------------------%
    
    for k1=1:length(obstacle(:,1))
        M1(k,4*k1-3)=obstacle(k1,1); % initial distance
        M1(k,4*k1-2)=obstacle(k1,2); % relative angle
        M1(k,4*k1-1)=obstacle(k1,5); % obstacle height
        M1(k,4*k1)=obstacle(k1,6);   % obstacle speed
    end
TIMES = [TIMES; a(k), toc];
end
% toc
%------------------------------------------------------------------------%
    %newpath=avoidance(obstacle);
    %---------------start statistic of the successful rate---------------%
    b=0;               % account for total successful cases
    b0=0; bs0=0;       % b0 accounts for non-obstacles. bs0 accounts for the response successful cases
    b1=0; bs1=0;       % b1 accounts for obstacle number=1. bs1 accounts for the response successful cases
    b2=0; bs2=0;       % b2 accounts for obstacles number=2. bs2 accounts for the response successful cases
    b3=0; bs3=0;       % b3 accounts for obstacles number=3. bs3 accounts for the response successful cases
    b4=0; bs4=0;       % b4 accounts for obstacles number>=4. bs4 accounts for the response successful cases
    b5=0; bs5=0;
    b6=0; bs6=0;
    b7=0; bs7=0;
    b8=0; bs8=0;
    b9=0; bs9=0;
    b10=0; bs10=0;
    b11=0; bs11=0;
    b12=0; bs12=0;
    b13=0; bs13=0;
    b14=0; bs14=0;
    b15=0; bs15=0;
    
    
    k2=1;
    for i=1:n
        if a(i)==0     %--------statistic for non-obstacle cases---------%
           b0=b0+1; b=b+1;
           if Uthetav(i)==0
              bs0=bs0+1;
           end
        end
        if a(i)==1     %--------statistic for obstacle=1 cases---------%
           b1=b1+1;
             if mm(i)>=h*dm
                bs1=bs1+1; b=b+1;
             end
        end
        if a(i)==2      %--------statistic for obstacle=2 cases---------%
           b2=b2+1;
             if mm(i)>=h*dm
                bs2=bs2+1; b=b+1;
             end
        end 
        if a(i)==3       %--------statistic for obstacle=3 cases---------%
           b3=b3+1;
             if mm(i)>=h*dm
                bs3=bs3+1; b=b+1;
             end            
        end
        if a(i)==4       %--------statistic for obstacle>=4 cases--------%
           b4=b4+1;
             if mm(i)>=h*dm
                bs4=bs4+1; b=b+1;
             end
        end 
        
        if a(i)==5       %--------statistic for obstacle=5 cases--------%
           b5=b5+1;
             if mm(i)>=h*dm
                bs5=bs5+1; b=b+1;
             end
        end 
        
        if a(i)==6       %--------statistic for obstacle=5 cases--------%
           b6=b6+1;
             if mm(i)>=h*dm
                bs6=bs6+1; b=b+1;
             end
        end 
        
        if a(i)==7       %--------statistic for obstacle=5 cases--------%
           b7=b7+1;
             if mm(i)>=h*dm
                bs7=bs7+1; b=b+1;
             end
        end
        
        if a(i)==8       %--------statistic for obstacle=5 cases--------%
           b8=b8+1;
             if mm(i)>=h*dm
                bs8=bs8+1; b=b+1;
             end
        end
        
        if a(i)==9       %--------statistic for obstacle=5 cases--------%
           b9=b9+1;
             if mm(i)>=h*dm
                bs9=bs9+1; b=b+1;
             end
        end
        
        if a(i)==10       %--------statistic for obstacle=5 cases--------%
           b10=b10+1;
             if mm(i)>=h*dm
                bs10=bs10+1; b=b+1;
             end
        end
        
        if a(i)==11       %--------statistic for obstacle=5 cases--------%
           b11=b11+1;
             if mm(i)>=h*dm
                bs11=bs11+1; b=b+1;
             end
        end
        
        if a(i)==12       %--------statistic for obstacle=5 cases--------%
           b12=b12+1;
             if mm(i)>=h*dm
                bs12=bs12+1; b=b+1;
             end
        end
        
        if a(i)==13       %--------statistic for obstacle=5 cases--------%
           b13=b13+1;
             if mm(i)>=h*dm
                bs13=bs13+1; b=b+1;
             end
        end
        
        if a(i)==14       %--------statistic for obstacle=5 cases--------%
           b14=b14+1;
             if mm(i)>=h*dm
                bs14=bs14+1; b=b+1;
             end
        end
        
        if a(i)==15       %--------statistic for obstacle=5 cases--------%
           b15=b15+1;
             if mm(i)>=h*dm
                bs15=bs15+1; b=b+1;
             end
        end 
        
    end
    successful_rate=b/k  % total rate
    successful_rate0=bs0/b0  % obstacle free successful rate
    successful_rate1=bs1/b1  % avoid one rate
    successful_rate2=bs2/b2  % avoid two rate
    successful_rate3=bs3/b3  % avoid three rate
    successful_rate4=bs4/b4  % avoid greater than four rate
    successful_rate5=bs5/b5  % avoid three rate
    successful_rate6=bs6/b6  % avoid greater than four rate
    successful_rate7=bs7/b7  % obstacle free successful rate
    successful_rate8=bs8/b8  % avoid one rate
    successful_rate9=bs9/b9  % avoid two rate
    successful_rate10=bs10/b10  % avoid three rate
    successful_rate11=bs11/b11  % avoid greater than four rate
    successful_rate12=bs12/b12 % avoid three rate
    successful_rate13=bs13/b13  % avoid greater than four rate
    successful_rate14=bs14/b14  % avoid greater than four rate
    successful_rate15=bs15/b15 % avoid three rate

meanTFGA = zeros(1, 15);
for t = 1:15
meanTFGA(t) =  mean(TIMES(find(TIMES(:, 1) == t), 2));
end
    
%------------------------------------------------------------------------%
%            if abs(Uthetav(i))<1.0
%               bs1=bs1+1; b=b+1;
%            elseif mm(i)>=h*dm
%               bs1=bs1+1; b=b+1; 
%            else
%               M2(k2,:)=M1(i,:);
%               k2=k2+1;
%            end

%            if abs(Uthetav(i))<1.0
%               bs2=bs2+1; b=b+1;
%            elseif mm(i)>=h*dm
%               bs2=bs2+1; b=b+1;
%            else
%               M2(k2,:)=M1(i,:);
%               k2=k2+1;
%            end

%            if abs(Uthetav(i))<1.0
%               bs3=bs3+1; b=b+1;
%            elseif mm(i)>=h*dm
%               bs3=bs3+1; b=b+1; 
%            else
%               M2(k2,:)=M1(i,:);
%               k2=k2+1;
%            end

%            if abs(Uthetav(i))<1.0
%               bs4=bs4+1; b=b+1;
%            elseif mm(i)>=h*dm
%               bs4=bs4+1; b=b+1;
%            else
%               M2(k2,:)=M1(i,:);
%               k2=k2+1;
%            end