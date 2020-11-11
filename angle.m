function theta=angle(x1,y1,x2,y2)

x=x2-x1;
y=y2-y1;
 
theta=180-90*(1+sign(x))* (1-sign(y^2))-45*(2+sign(x)) *sign(y)-(180/pi())*sign(x*y)*atan((abs(x)-abs(y))/(abs(x)+abs(y)));
 
end