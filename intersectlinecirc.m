function [IX1,IX2] = intersectlinecirc(h,k,a,x1,y1,x2,y2);

centre=[h k]
radius=a

m=(y2-y1)/(x2-x1)
c=(-y2*x1+x2*y1)/(x2-x1)

A=1+m^2;
B=2*m*c-2*h-2*k*m;
C=c^2-2*k*c+h^2+k^2-a^2;

D=B^2-4*A*C;

if D>0
    
    disp('intersection at two points');
    
    X1=(-B+sqrt(D))/(2*A);
    Y1=m*X1+c;
   
    X2=(-B-sqrt(D))/(2*A);
    Y2=m*X2+c;
    
    IX1=[X1 Y1];
    IX2=[X2 Y2];
    
    return
    
elseif D==0
    
    disp('intersection at single points');
    
    X1=-B/2*A;
    Y1=m*X1+c;
    
    IX1=[X1 Y1];
    IX2=[];
    
    return
    
else
    
    disp('No intersection points');
    
    
    IX1=[];
    IX2=[];

return
end