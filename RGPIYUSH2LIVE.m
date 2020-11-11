clearvars;

close all
clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%% vision module
rR=[0  0]        %%%%%%%%% robot position in x and y direction
rRx=rR(1);
rRy=rR(2);


rT=[30 0]      %%%%%%%%% target position in x and y direction
rTx=rT(1);
rTy=rT(2);

t=0.010;

vR=[0 0];     %%%%%%%%% robot initial velocity vR in x and y direction
vRx=vR(1);
vRy=vR(2);
magvR=sqrt((vRx)^2+(vRy)^2);

magvT=85;    %%%%%%%%%%% defining target velocity vT magnitude and angle
thetaT=70;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% positioning vT

vTx=rT(1)+magvT*cos((pi/180)*thetaT);
vTy=rT(2)+magvT*sin((pi/180)*thetaT);

vT=[vTx vTy];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%profs idea of positioning vT

%vT=[80 80];

%vT=[80 80] + rT;
%vTx=vT(1);
%vTy=vT(2);

%magT=3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Amax=[8 0];
magAmax=sqrt(((Amax(1))^2)+(Amax(2)^2));


drTvT=sqrt((vTx-rTx)^2+(vTy-rTy)^2); %distance between rT and vT
dRT=sqrt((rTx-rRx)^2+(rTy-rRy)^2); %distance between position of robot rR and target rT



%vT=[10,7] assumed vTR
%vTRx=-10;
%vTRy=7;
%%%%%%%%%%%%%%%%%%%%% making of the FAR
Aax=0;
Aay=8;
Abx=8;
Aby=0;
Acx=0;
Acy=-8;
Adx=-8;
Ady=0;

AX=[Aax,Abx,Acx,Adx];
AY=[Aay,Aby,Acy,Ady];

FAR=fill(AX,AY,'y')
transparency=0.2;
alpha(transparency);

hold on

%%%%%%%%%%%%%%%%% to check if Amax lies in FAR 
[in,on]=inpolygon(Amax(1),Amax(2),AX,AY);

 if in == 1 && on == 1
    
    disp('edge of the FAR')
    
    
  elseif in == 1
    
    disp('Inside the FAR')

   else
    
    disp('outside the FAR,choose a feasible accelration')
    
    pAmax=plot(Amax(1),Amax(2),'o','markerfacecolor','g');hold on;
    break
 end
 i=1
for i=1:100
    
    %t=t+t;
    
    rR=[rR(1) rR(2)]
    rT=[rT(1) rT(2)]
    vR=[vR(1) vR(2)]
    vT=[vT(1) vT(2)]
    
    dRT=sqrt((rT(1)-rR(1))^2+(rT(2)-rR(2))^2);
    
%%%%%%%%%%%%%%%%%%%%% making of the FVR
vax=vRx+Aax*t;
vbx=vRx+Abx*t;
vcx=vRx+Acx*t;
vdx=vRx+Adx*t;
vay=vRy+Aay*t;
vby=vRy+Aby*t;
vcy=vRy+Acy*t;
vdy=vRy+Ady*t;

FVRX=[vax,vbx,vcx,vdx];
FVRY=[vay,vby,vcy,vdy];

pFVR=fill(FVRX,FVRY,'r');hold on
transparency=0.2;
alpha(transparency);

%%%%%%%%%%%%%%%%% to check if v lies in FVR

%[in,on]=inpolygon(vR(1),vR(2),FVRX,FVRY);
%
% if in == 1 && on == 1
%    
%    disp('edge of the FVR')
%    
%    
%  elseif in == 1
%    
%    disp('Inside the FVR')
%
%   else
%    
%    disp('outside the FVR,choose a feasible velocity from the red')
%    
%    pvR=plot(vR(1),vR(2),'o','markerfacecolor','g');hold on;
%    break
%end


%%%%%%%%%%%%%%%%%%%%%%%% angles thetaLOS,thetaT

%thetaLOS=atan((rTy-rRy)/(rTx-rRx));
%hetaLOSdeg=((180/pi)*atan((rTy-rRy)/(rTx-rRx)));
%thetaT=atan((vTy-rTy)/(vTx-rTx));       
%thetaTdeg=((180/pi)*atan((vTy-rTy)/(vTx-rTx)));
thetaT=angle(rT(1),rT(2),vT(1),vT(2));
thetaLOS=angle(rR(1),rR(2),rT(1),rT(2));

%%%%%%%%%%%%%%%%%%%%%%%% positioning vRT and vT

%vRTx=rRx+drTvT*(cos(thetaT-thetaLOS));
%vRTy=rRy+drTvT*(sin(thetaT-thetaLOS));
%vRT=[vRTx vRTy];

%vRTx=rRx+drTvT*(cos((pi/180)*(thetaT+thetaLOS)));
%vRTy=rRy+drTvT*(sin((pi/180)*(thetaT+thetaLOS)));
%vRT=[vRTx vRTy];

%vTx=rTx+drTvT*cos(thetaT-thetaLOS);
%vTy=rTy+drTvT*sin(thetaT-thetaLOS);
%vT=[vTx vTy];

vRTx=rRx+drTvT*(cos((pi/180)*thetaT));
vRTy=rRy+drTvT*(sin((pi/180)*thetaT));
vRT=[vRTx vRTy];

magvRT=sqrt(vRT(1)^2+vRT(2)^2);

%%%%%%%%%%%%%%%%%%%%%%%%% calculating vrendmaxrel

magvrendmaxrel=sqrt(2*dRT*magAmax);

%rendmaxcircle=circle(rRx,rRy,magvrendmaxrel);hold on;

%%%%%%%%%%%%%%%%%% calculating the maximum robot velocity vRTmax circle

vRTmaxx=vT(1)+(magvrendmaxrel/dRT)*(rR(1)-rT(1));
vRTmaxy=vT(2)+(magvrendmaxrel/dRT)*(rR(2)-rT(2));

vRTmax=[vRTmaxx vRTmaxy];

magvRTmax=sqrt((vRTmaxx)^2+(vRTmaxy)^2);

%vRTmaxcircle=circle(rRx,rRy,magvRTmax);hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%intersection of circle and the RS line

%[A,B]=intersectlinecirc(rR(1),rR(2),magvRTmax,vRT(1),vRT(2),vT(1),vT(2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%intersection of FVR and the RS line

 [SP1,SP2]=squarelineintercept(vRTx,vRTy,vRTmaxx,vRTmaxy,vax,vay,vbx,vby,vcx,vcy,vdx,vdy)
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Assigning 0,0 to the empty coordinate

    if isempty(SP2)==1
        
        SP2=[0,0];
        
    else
        SP2=SP2;
        
    end
    
       if isempty(SP1)==1
        
        SP1=[0,0];
        
    else
        SP1=SP1
        
       end
    
       %%%%%%%%%%%%%%%%%%%%%choosing the final velocity
 
        
       
   dSP1vRT=sqrt((vRT(1)-SP1(1))^2+(vRT(2)-SP1(2))^2);
   
   dSP1vRTmax=sqrt((vRTmax(1)-SP1(1))^2+(vRTmax(2)-SP1(2))^2);
   
   dSP2vRT=sqrt((vRT(1)-SP2(1))^2+(vRT(2)-SP2(2))^2);
   
   dSP2vRTmax=sqrt((vRTmax(1)-SP2(1))^2+(vRTmax(2)-SP2(2))^2);
   
 dvRTvRTmax=sqrt((vRTmax(1)-vRT(1))^2+(vRTmax(2)-vRT(2))^2);  
 dvRTSP1vRTmax=dSP1vRT+dSP1vRTmax;
 dvRTSP2vRTmax=dSP2vRT+dSP2vRTmax;
   
 A1=round(dvRTvRTmax,3);
 A2=round(dvRTSP1vRTmax,3);
 A3=round(dvRTSP2vRTmax,3);  
           
   if A2==A1 %dvRTSP1vRTmax==dvRTvRTmax
       
       disp('SP1 lies inside seg RS');
       
       vRfinal=SP1
       %vRfinal1=SP1
   else
       
       disp('SP1 lies outside seg RS');
       SP1=[];
      
   end
   
   if A3==A1 %dvRTSP2vRTmax==dvRTvRTmax
       
       disp('SP2 lies inside seg RS');
       
       vRfinal=SP2
       %vRfinal2=SP2
       
   else
       
       disp('SP2 lies outside seg RS');
       SP2=[];
       
   end
   
   if ~isempty(SP1)==1 && ~isempty(SP2)==1
       
       disp('SP1 & SP2 lies inside seg RS');
       
       if round(dSP1vRT,3) < round(dSP2vRT,3)
           
           disp('SP2 is selected being bigger than SP1');
           
           vRfinal=SP2
           %vRfinal3=SP2
           
           SP1=[];
           
       else
           disp('SP1 is selected being bigger than SP1');
           
           vRfinal=SP1
           %vRfinal4=SP1
           
           SP2=[];
           
       end
       
   else
      if isempty(SP1)==1 && isempty(SP2)==1
           
           disp('vRTmax is the final velocity')
           
           vRfinal=vRTmax
           %vRfinal5=vRTmax
           
       else
       end
   end
   
%%%%%%%%%%%%%%%%%%%%when vRTmax inside the FVR

[in,on]=inpolygon(vRTmax(1),vRTmax(2),FVRX,FVRY);

if in==1 && on==1
    
    disp('vRTmax on the edge of FVR');
    
    disp('vRTmax must be selected');
    
     vRfinal=vRTmax
    %vRfinal6=vRTmax
    
elseif in==1
    
    disp('vRTmax inside the FVR');
    
    disp('vRTmax must be selected');
    
    vRfinal=vRTmax
    %vRfinal7=vRTmax
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 i
RX(1)=rR(1);
RY(1)=rR(2);
TX(1)=rT(1);
TY(1)=rT(2);

RX(i+1)=vRfinal(1)*t+RX(i);
RY(i+1)=vRfinal(2)*t+RY(i);
TX(i+1)=vTx*t+TX(i);
TY(i+1)=vTy*t+TY(i);

rRx=round(RX(i+1),2)
rRy=round(RY(i+1),2)
rTx=round(TX(i+1),2)
rTy=round(TY(i+1),2)

    i=i+1;
    



%%%%%%%%%%%%%%%%%%%%%%%%%%making of plots


 subplot(2,1,1)
 xlabel('[m]','interpreter','latex'); ylabel('[m]','interpreter','latex');hold on;
%poo=plot(0,0,'o','Markeredgecolor','r');hold on;
pR=plot(rRx,rRy,'o','Markerfacecolor','r','Markeredgecolor','r','Markersize',5);hold on;
pT=plot(rTx,rTy,'o','Markerfacecolor','b','Markeredgecolor','b','Markersize',5);hold on;
axis([0 1000 0 1000])
drawnow limitrate;
%vR=plot(vRx,vRy,'^','Markerfacecolor','r','Markeredgecolor','b');hold on;
%pvT=plot(vTx,vTy,'s','Markerfacecolor','b','Markeredgecolor','b');hold on;
%pvRT=plot(vRTx,vRTy,'s','Markerfacecolor','r','Markeredgecolor','b');hold on;
%pvRTmax=plot(vRTmax(1),vRTmax(2),'^','Markerfacecolor','r','Markeredgecolor','b')
%vTR=plot(vTRx,vTRy,'s','Markerfacecolor','k');hold on;



%%%% making LOS and RL;
%pLOS=plot([rR(1) rT(1)] , [rR(2) rT(2)],'c','linewidth',2);hold on;
%pRL=plot([vRT(1) vT(1)], [vRT(2) vT(2)],'--m','linewidth',1);hold on;
%pRS=plot([vRTmax(1) vRT(1)], [vRTmax(2) vRT(2)],'m','linewidth',4);hold on;
%pvRTo=plot([vRT(1) 0], [vRT(2) 0],':k');hold on;
%pRo=plot([rR(1) 0], [rR(2) 0],':k');hold on;
%ptTvT=plot([rT(1) vT(1)],[rT(2) vT(2)],':k');hold on;
%pvRTmaxrR=plot([rR(1) vRTmax(1)],[rR(2) vRTmax(2)],':k');hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%if SP1 exist plot and SP2 exist plot
if (isempty(SP1)~=1)
pSP1=plot(SP1(1),SP1(2),'o','Markerfacecolor','g','Markeredgecolor','r');hold on;
else
end

if (isempty(SP2)~=1)
pSP2=plot(SP2(1),SP2(2),'o','Markerfacecolor','g','Markeredgecolor','b');hold on;
else
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



hold  off

end
   
