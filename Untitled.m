%h=circle(1,2,9)

%%%%%%%% to make a polygon if the four co ordinates of its are known to you

x=[0,1,-2.5,-1.5];
y=[1,0,-1.5,-0.5];

fill(x,y,'b')

%%%%%%%% to check if a point lies inside or outside of the polygon

[in,on]=inpolygon(10,10,x,y);

  if in == 1 && on == 1
    
    disp('edge of the FVR')
    
    
  elseif in == 1
    
    disp('Inside the FVR')

   else
    
    disp('outside the FVR')
    break
    
  end

  disp('happy')