function R = IsPointWithinLine(x1, y1, x2, y2, x3, y3)
% Line equation: y = m*x + b;
Limit = 100 * eps(max(abs([x1,y1,x2,y2,x3,y3])));
if x1 ~= x2
  m   = (y2-y1) / (x2-x1);
  yy3 = m*x3 + y1 - m*x1;
  R   = (abs(y3 - yy3) < 100 * Limit);
else
  R   = (x3 < Limit);
end