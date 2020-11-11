rRx=0;% pursuer position
rRy=0;% pursuer velocity with coloumns as vx and vy.
vRx=0;
vRy=0;

r01=[950,950];% obstacle1 position
v01=[0,0];% obstacle1 velocity

r01x=950;
r01y=950;
v01x=0;
v01y=0;
x01=r01x;
y01=r01y;

r02=[550,650];% obstacle2 position 
v02=[0,0];% obstacle2 velocity
r02x=550;
r02y=650;
v02x=0;
v02y=0;
x02=r02x;
y02=r02y;

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


rR2T=sqrt((rTx-rRx)^2+(rTy-rRy)^2);
Vrel=sqrt((vTx-vRx)^2+(vTy-vRy)^2);


VOx=v01x;
VOy=v01y;

%%[~,vRxv1, vRyv1]= RGGuidance(rRx,rRy,rTx,rTy,vRx,vRy,vTx,vTy);
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
