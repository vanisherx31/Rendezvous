function X=intersectline(x1,y1,x2,y2,x3,y3,x4,y4);

 m1=(y2-y1)/(x2-x1)
 c1=(x2*y1-x1*y2)/(x2-x1)

  m2=(y4-y3)/(x4-x3)
  c2=(x4*y3-x3*y4)/(x4-x3)
  
  x=(c2-c1)/(m2-m1)
  
  y=m1*x+c1
  
  X=[x y]
end
 