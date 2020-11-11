clearvars; 
close all;
clear all;

rRx=0;% pursuer position
rRy=0;% pursuer velocity with coloumns as vx and vy.
vRx=0;
vRy=0;

%r01=[4000,1150];% obstacle1 position
%v01=[0,0];% obstacle1 velocity

%r01x=4000;
%r01y=1150;
%v01x=0;
%v01y=0;
%x01=r01x;
%y01=r01y;

%r02=[3000,1050];% obstacle2 position 
%v02=[0,0];% obstacle2 velocity
%r02x=3000;
%r02y=1050;
%v02x=0;
%v02y=0;
%x02=r02x;
%y02=r02y;

rT=[300,900];% targets position
vT=[0,0];% targets velocity
rTx=rT(1);
rTy=rT(2);
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
while(rR2T>15)
  


    rR2T=sqrt((rTx-rRx)^2+(rTy-rRy)^2)  
%    r01=sqrt((x01-rRx)^2+(y01-rRy)^2)
%    r02=sqrt((x02-rRx)^2+(y02-rRy)^2)
    
%    if (r01<50) || (r02<50)
%        disp('collision happening');
%        break;
    
    
  
    R=15;  % radius of the obstacle+radius of the robot
%    x01p=sqrt(r01^2-R^2);% distance between robot and obstacle periphery
%    anglecup01=180*atan(R/x01p)/pi;%angle between centre(line joining robot and obstacle) and upper periphery
%   anglehoc01=180*atan((y01-rRy)/(x01-rRx))/pi;%angle between horizontal x-axis and centre line joining robot and obstacle
%     
%     %%***********%%
%    angUP01=anglecup01+anglehoc01;
%    angDWN01=anglehoc01-anglecup01;
%     
%   x02p=sqrt(r02^2-R^2);% distance between robot and obstacle periphery
%    anglecup02=180*atan(R/x02p)/pi;%angle between centre(line joining robot and obstacle) and upper periphery
%    anglehoc02=180*atan(y02-rRx/(x02-rRy))/pi;%angle between horizontal x-axis and centre line joining robot and obstacle
     
%     %%****************%% 
%    angUP02=anglecup02+anglehoc02;
%    angDWN02=anglehoc02-anglecup02;
%     
     %%distance between robo and obstacles
  
    vrel01=((vRx-100)^2+(vRy-100)^2);
    vrel02=((vRx-100)^2+(vRy-100)^2);
%     
%     %%time to collide
%    t01=r01/vrel01;
%    t02=r02/vrel02;
%    if t01<t02
%        tVO=t01;
%    else
%        tVO=t02;
%    end
    
%     %%current distance between robot and the target
    
    Vrel=sqrt((vTx-vRx)^2+(vTy-vRy)^2);
   
    %88888888888888888888888888888888888**********
     
%     tVO    
 %   if tVO<tH


%        [vRx2,vRy2] = pureVO(angUP01, angDWN01,angUP02, angDWN02,rRx,rRy,rTx,rTy,vRx,vRy,vTx,vTy);
%        RX(i+1)=vRx2*.1 + RX(i);
%        RY(i+1)=vRy2*.1 + RY(i);
%        disp('pureVO block vel');
%        vRy=vRy2
%        vRx=vRx2
%        rRx=RX(i+1);
%        rRy=RY(i+1);
%        disp('so im in pureVO block');
           
%   else
        
        r=sqrt((rTx-rRx)^2+(rTy-rRy)^2);                    %distance between robot and target calculated for every next time instant  
maxacc=30;                                                %maximum acceleration of robot
Vrmax=sqrt(2*r*maxacc);                                     %maximum velocity of robot

alpha1=(150-vTx)/(r*cos(180*atan((rTy-rRy)/(rTx-rRx))/pi));  %alpha1=ratio of relative velocity of robot w.r.t to target in the x direction to the distance between robot and target in the x direction at every next time instant 
alpha2=(150-vTy)/(r*sin(180*atan((rTy-rRy)/(rTx-rRx))/pi));  %alpha1=ratio of relative velocity of robot w.r.t to target in the y direction to the distance between robot and target in the y diraction at every next time instant

alpha=sqrt(alpha1^2+alpha2^2)

    
vRxv1= vTx+alpha*r*cos(180*atan((rTy-rRy)/(rTx-rRx))/pi);   %velocity of robot in the x direction
vRyv1=vTy+alpha*r*sin(180*atan((rTy-rRy)/(rTx-rRx))/pi);    %velocity of robot in the y direction
Vabs=sqrt(vRxv1^2+vRyv1^2);                                 %resultant velocity of robot which moves the robot closer to the target
Vrmax_x = Vrmax*cos(180*atan(vRyv1/vRxv1)/pi) ;             %x-component of maximum velocity of robot
Vrmax_y = Vrmax*sin(180*atan(vRyv1/vRxv1)/pi) ;             %y-component of maximum velocity of robot

if Vabs<Vrmax || Vrmax>92.1301 || Vrmax<10 
    vRx=vRxv1; 
    vRy=vRyv1;
else 
    vRx=Vrmax_x; 
    vRy=Vrmax_y;
end

        
        %i;
        RX(i+1)=vRxv1*.01 + RX(i);
        RY(i+1)=vRyv1*.01 + RY(i);
        vRy=vRyv1;
        vRx=vRxv1;
        rRx=RX(i+1);
        rRy=RY(i+1);
        disp('so im in rg block');
    %end
          
          
        
    
    %x01=x01+19*.01;
    %x02=x02-120*.01;
    %y01=y01+19*.01;
    %y02=y02+120*.01;
    Ty=Ty+90*.01;
    Tx=Tx+90*.01;
    rTx=Tx;
    rTy=Ty;
   
    subplot(2,1,1)
    hold on;
    hold on;
    p13=plot(rRx,rRy,'o','MarkerFaceColor','k','MarkerEdgeColor','none');
    p14= plot(Tx,Ty,'o','MarkerFaceColor','b','MarkerEdgeColor','none');
    hold off;
    drawnow limitrate
   
  
   i = i+1;
    if i>3000
        break;
    end
    
   
end
 