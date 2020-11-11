function R = isPointOnLine(P1, P2, Q, EndPoints)
% Is point Q=[x3,y3] on line through P1=[x1,y1] and P2=[x2,y2]
% Normal along the line:
P12 = P2 - P1;
L12 = sqrt(P12 * P12');
N   = P12 / L12;
% Line from P1 to Q:
PQ = Q - P1;
% Norm of distance vector: LPQ = N x PQ
Dist = abs(N(1) * PQ(2) - N(2) * PQ(1));
% Consider rounding errors:
Limit = 10 * eps(max(abs(cat(1, P1(:), P2(:), Q(:)))));
R     = (Dist < Limit);
% Consider end points if any 4th input is used:
if R && nargin == 4
  % Projection of the vector from P1 to Q on the line:
  L = PQ * N.';  % DOT product
  R = (L > 0.0 && L < L12);
end