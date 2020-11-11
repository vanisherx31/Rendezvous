%code for dynamic obstacle added following PURE RG strategy and no VO strategy considered 

clearvars; 
close all;
clear all;

pos=zeros(6001,4);

rRx=0;% pursuer position
rRy=0;% pursuer velocity with coloumns as vx and vy.
vRx=700;
vRy=700;
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
    
rR2T=sqrt((rTx-rRx)^2+(rTy-rRy)^2)  
r=sqrt((rTx-rRx)^2+(rTy-rRy)^2);                            %distance between robot and target calculated for every next time instant  
maxacc=9000;                                                %maximum acceleration of robot
Vrmax=sqrt(2*r*maxacc)                                     %maximum velocity of robot

alpha1=(vRx-vTx)/(r*cos(180*atan((rTy-rRy)/(rTx-rRx))/pi));  %alpha1=ratio of relative velocity of robot w.r.t to target in the x direction to the distance between robot and target in the x direction at every next time instant 
alpha2=(vRy-vTy)/(r*sin(180*atan((rTy-rRy)/(rTx-rRx))/pi));  %alpha1=ratio of relative velocity of robot w.r.t to target in the y direction to the distance between robot and target in the y diraction at every next time instant

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
        RX(i+1)=vRxv1*.01 + RX(i)
        RY(i+1)=vRyv1*.01 + RY(i)
        %vRy=vRyv1
        %vRx=vRxv1
        rRx=RX(i+1);
        rRy=RY(i+1);
        
        disp('so im in rg block');
   
                  
          
    % adding motion to the obstacle1 and obstacle2    
    
    x01=x01+19*.01;   % adding motion to the obstacle1 along x-direction
    y01=y01+509*.01;   % adding motion to the obstacle1 along y-direction
    
    x02=x02+190*.01;   % adding motion to the obstacle2 along x-direction
    y02=y02+620*.01    % adding motion to the obstacle2 along y-direction
   
    % adding motion to the target
    
    Tx=Tx-250*.01       % adding motion to the target along x-direction
    Ty=Ty-320*.01     % adding motion to the target along y-direction
    
    rTx=Tx;
    rTy=Ty;
   
    subplot(2,1,1)
    p11=plot(x01,y01,'s','MarkerFaceColor','r','MarkerEdgeColor','none','Markersize',15);
    xlabel(' [m]','interpreter','latex'); ylabel('[m]','interpreter','latex');hold on;
    p12=plot(x02,y02,'s','MarkerFaceColor','y','MarkerEdgeColor','none','Markersize',15);
    xlabel('$obstacle02x$ [m]','interpreter','latex'); ylabel('$obstacle02y$ [m]','interpreter','latex');hold on;
    p13=plot(rRx,rRy,'o','MarkerFaceColor','r','MarkerEdgeColor','none');
    p14= plot(rTx,rTy,'o','MarkerFaceColor','b','MarkerEdgeColor','none');
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