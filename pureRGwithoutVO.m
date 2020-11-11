%code for dynamic obstacle added following PURE RG strategy and no VO strategy considered 

clearvars; 
close all;
clear all;

pos=zeros(6001,4);

rRx=0;% pursuer position
rRy=0;% pursuer velocity with coloumns as vx and vy.
vRx=0;
vRy=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% adding of the obstacles

r01=[950,950];% obstacle1 position
v01=[0,0];% obstacle1 velocity

r01x=950;
r01y=950;
v01x=0;
v01y=0;
x01=r01x;
y01=r01y;

r02=[450,450];% obstacle2 position 
v02=[0,0];% obstacle2 velocity
r02x=450;
r02y=450;
v02x=0;
v02y=0;
x02=r02x;
y02=r02y;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rT=[3000,3000];% targets position
vT=[0,0];% targets velocity
rTx=3000;
rTy=3000;
vTx=0;
vTy=0;
Ty=rTy;
Tx=rTx;



i=1;
RX(1)=rRx;
RY(1)=rRy;



tH=.002;
rR2T=sqrt((rTx-rRx)^2+(rTy-rRy)^2);
Vrel=sqrt((vTx-vRx)^2+(vTy-vRy)^2);
i=1;

while(rR2T>25)
    
rR2T=sqrt((rTx-rRx)^2+(rTy-rRy)^2) %istance between the robo and the target at every instant  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% adding of the obstacles

r01=sqrt((x01-rRx)^2+(y01-rRy)^2) %distance between cetnre line of the robo and the OB1
r02=sqrt((x02-rRx)^2+(y02-rRy)^2) %distance between cetnre line of the robo and the OB2
    
    %if (r01<400) || (r02<300)
    %   disp('collision happening');
    %    break;
    %%%%%%%%%%%end
    
        R=25;  % radius of the obstacle+radius of the robot
    x01p=sqrt(r01^2-R^2);% distance between robot and obstacle periphery
    anglecup01=180*atan(R/x01p)/pi;%angle between centre(line joining robot and obstacle) and upper periphery
    anglehoc01=180*atan((y01-rRy)/(x01-rRx))/pi;%angle between horizontal x-axis and centre line joining robot and obstacle
%     
%     %%***********%%
    angUP01=anglecup01+anglehoc01
    angDWN01=anglehoc01-anglecup01
%     
    x02p=sqrt(r02^2-R^2);% distance between robot and obstacle periphery
    anglecup02=180*atan(R/x02p)/pi;%angle between centre(line joining robot and obstacle) and upper periphery
    anglehoc02=180*atan(y02-rRx/(x02-rRy))/pi;%angle between horizontal x-axis and centre line joining robot and obstacle
     
%     %%****************%% 
    angUP02=anglecup02+anglehoc02
    angDWN02=anglehoc02-anglecup02
%     
     %%distance between robo and obstacles
  
    vrel01=((vRx-19)^2+(vRy-509)^2)
    vrel02=((vRx+190)^2+(vRy-620)^2)
%     
%     %%time to collide
    t01=r01/vrel01
    t02=r02/vrel02
    
    if t01<t02
        tVO=t01;
    else
        tVO=t02;
    end
    
    Vrel=sqrt((vTx-vRx)^2+(vTy-vRy)^2)
    
    tVO
     if tVO<tH
         VOx=v01x;
VOy=v01y;

[~,vRxv1, vRyv1]= RGGuidance(rRx,rRy,rTx,rTy,vRx,vRy,vTx,vTy);
zeta=180*atan(vRyv1/vRxv1)/pi;
%%*****STEP0*****%%


angUP=angUP01;
angDWN=angDWN01;



% %%%%%%     *****X3******
%            *           *
%            *           *
%            *           *
%            Y1          Y2
%           0 *           *
%            *           *
%            *****X4******

%%STEP1%%

 Y1=[200,(200-VOx)*tan(angUP)+VOy];
 Y2=[300,(300-VOx)*tan(angUP)+VOy];
 X3=[((300-VOy)/tan(angUP))+VOx,300];
 X4=[((200-VOy)/tan(angUP))+VOx,200];
 
 Y21=[200,(200-VOx)*tan(angDWN)+VOy];
 Y22=[300,(300-VOx)*tan(angDWN)+VOy];
 X23=[((300-VOy)/tan(angDWN))+VOx,300];
 X24=[((200-VOy)/tan(angDWN))+VOx,200];
 
%%STEP2%% 
 verify=[Y1;Y2;X3;X4];
 i=1;
 
 if i<5
     k=1;
     if k<3
         
        if verify(i,k)>=200 && verify(i,k)<=300
            j=1;
            tempwhich(j)=i;
            j=j+1;
        end
        k=k+1;
     end
      
 end
 
 verify2=[Y21;Y22;X23;X24];
 i2=1;
 
 if i2<5
     k2=1;
     if k2<3
         
        if verify(i2,k2)>=200 && verify(i2,k2)<=300
            j2=1;
            tempwhich2(j2)=i;
            j2=j2+1;
        end
        k2=k2+1;
     end
      
 end
 
 %%STEP3%%
m=tan(angUP); %SLOPE OF THE VO LINE
[~,vRxv1, vRyv1]= RGGuidance(rRx,rRy,rTx,rTy,vRx,vRy,vTx,vTy); 

if (vRyv1-VOy-m*(vRxv1-VOx))==0 
     vRx=vRxv1;
     vRy=vRyv1;
else
    VIP1 = verify(tempwhich(1),:); 
    VIP2 = verify(tempwhich(2),:);
    
    rRx1 = rRx+.1*VIP1(1,1);
    rRy1 = rRy+.1*VIP1(1,2);
    rRx2 = rRx+.1*VIP2(2,1);
    rRy2 = rRy+.1*VIP2(2,2);
    
    rVIP1= sqrt((rTy-rRy1)^2+(rTx-rRx1)^2);
    rVIP2= sqrt((rTy-rRy2)^2+(rTx-rRx2)^2);
end


    m2=tan(angDWN); %SLOPE OF THE VO LINE
    [~,vRxv1, vRyv1]= RGGuidance(rRx,rRy,rTx,rTy,vRx,vRy,vTx,vTy); 

if (vRyv1-VOy-m*(vRxv1-VOx))==0 
     vRx=vRxv1;
     vRy=vRyv1;
else
    VIP3 = verify2(tempwhich2(0),:); 
    VIP4 = verify2(tempwhich2(1),:);
    
    rRx3 = rRx+.1*VIP3(1,1);
    rRy3 = rRy+.1*VIP3(1,2);
    rRx4 = rRx+.1*VIP4(2,1);
    rRy4 = rRy+.1*VIP4(2,2);
    
    rVIP3= sqrt((rTy-rRy3)^2+(rTx-rRx3)^2);
    rVIP4= sqrt((rTy-rRy4)^2+(rTx-rRx4)^2);
end 

%% ** STEP5 **
 
dmin = [rVIP1,rVIP2,rVIP3,rVIP4];
p = 1;
temp = 0;
VIP = [VIP1;VIP2;VIP3;VIP4];

if p<3
    if(dmin(p+1)<dmin(p))
        temp=p+1;
    end
    
    p=p+1;
end

vRx=VIP(temp,1);
vRy=VIP(temp,2);
         disp('so im in pureVO block');
    
     else
     end        
    
        
%%%%%%%%%%distance between robot and target calculated for every next time instant
r=sqrt((rTx-rRx)^2+(rTy-rRy)^2); 
maxacc=9000;                                                %maximum acceleration of robot
Vrmax=sqrt(2*r*maxacc);                                     %maximum velocity of robot

alpha1=(1050-vTx)/(r*cos(180*atan((rTy-rRy)/(rTx-rRx))/pi));  %alpha1=ratio of relative velocity of robot w.r.t to target in the x direction to the distance between robot and target in the x direction at every next time instant 
alpha2=(1050-vTy)/(r*sin(180*atan((rTy-rRy)/(rTx-rRx))/pi));  %alpha1=ratio of relative velocity of robot w.r.t to target in the y direction to the distance between robot and target in the y diraction at every next time instant

%alpha=sqrt(alpha1^2+alpha2^2)
 if alpha1<alpha2
    alpha=alpha1;
    
else 
    alpha=alpha2;
end 
    
vRxv1= vTx+alpha*r*cos(180*atan((rTy-rRy)/(rTx-rRx))/pi);   %velocity of robot in the x direction
vRyv1=vTy+alpha*r*sin(180*atan((rTy-rRy)/(rTx-rRx))/pi);    %velocity of robot in the y direction
Vabs=sqrt(vRxv1^2+vRyv1^2);                                 %resultant velocity of robot which moves the robot closer to the target
Vrmax_x = Vrmax*cos(180*atan(vRyv1/vRxv1)/pi) ;             %x-component of maximum velocity of robot
Vrmax_y = Vrmax*sin(180*atan(vRyv1/vRxv1)/pi) ;             %y-component of maximum velocity of robot

if Vabs<Vrmax || Vrmax>595|| Vrmax<250 
    vRx=vRxv1; 
    vRy=vRyv1;
else 
    vRx=Vrmax_x; 
    vRy=Vrmax_y;
end

        
        i;
        RX(i+1)=vRxv1*.01 + RX(i);
        RY(i+1)=vRyv1*.01 + RY(i);
        vRy=vRyv1
        vRx=vRxv1
        rRx=RX(i+1);
        rRy=RY(i+1);
        
        disp('so im in rg block');
   
                  
          
    % adding motion to the obstacle1 and obstacle2    
    
    x01=x01+19*.01;   % adding motion to the obstacle1 along x-direction
    y01=y01+509*.01;   % adding motion to the obstacle1 along y-direction
    
    x02=x02-190*.01;   % adding motion to the obstacle2 along x-direction
    y02=y02+620*.01    % adding motion to the obstacle2 along y-direction
   
    % adding motion to the target
    
    Tx=Tx-250*.01;       % adding motion to the target along x-direction
    Ty=Ty-300*.01;     % adding motion to the target along y-direction
    
    rTx=Tx;
    rTy=Ty;
   
    subplot(2,1,1)
    p11=plot(x01,y01,'o','MarkerFaceColor','g','MarkerEdgeColor','none','Markersize',25);xlabel('$obstacle01x$ [m]','interpreter','latex'); ylabel('$obstacle01y$ [m]','interpreter','latex');hold on;
    p12=plot(x02,y02,'o','MarkerFaceColor','y','MarkerEdgeColor','none','Markersize',25);xlabel('$obstacle02x$ [m]','interpreter','latex'); ylabel('$obstacle02y$ [m]','interpreter','latex');hold on;
    p13=plot(rRx,rRy,'o','MarkerFaceColor','k','MarkerEdgeColor','none');
    p14= plot(Tx,Ty,'o','MarkerFaceColor','b','MarkerEdgeColor','none');
    hold off;
    drawnow limitrate
   
  
   i = i+1;
    if i>15000
        break;
    end
    
    PX=RX';
    PY=RY';
    pos=[PX,PY];
   %pos(6001,:)=PX; 
   %pos(6001,:)=PY;
   
   
end