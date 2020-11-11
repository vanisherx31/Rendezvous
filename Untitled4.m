%%%%%%%% to make a polygon if the four co ordinates of its are known to you

xpoly=[0,1,-0.5,-1.5];
ypoly=[1,0,-1.5,-0.5];
fill(xpoly,ypoly,'b')

[in,on]=checkpoly(0,0,xpoly,ypoly)
