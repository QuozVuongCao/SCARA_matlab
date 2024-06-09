function [T10 T20 T30 T40] = forward(theta_1, theta_2, d_3, theta_4)

% syms theta_1;
% syms theta_2;
% syms theta_4;
% syms d_3;

a_1=450;
a_2=400;
d_1=363;
d_4=-363; %-420

A10=[ cos(theta_1)   -sin(theta_1)      0    a_1*cos(theta_1);
      sin(theta_1)    cos(theta_1)      0    a_1*sin(theta_1);
      0                  0              1          d_1       ;
      0                  0              0           1       ];
A21=[ cos(theta_2)    -sin(theta_2)     0    a_2*cos(theta_2);
      sin(theta_2)     cos(theta_2)     0    a_2*sin(theta_2);
      0                 0               1           0        ;
      0                 0               0           1       ];
A32=[1  0	0	 0;
     0  1   0    0;
     0  0   1   d_3;
     0  0   0    1];
A43=[cos(theta_4)     sin(theta_4)      0      0;
     sin(theta_4)    -cos(theta_4)      0      0;
      0                 0              -1    d_4;
      0                 0               0     1];
  
T10=A10;
T20=T10*A21;
T30=T20*A32;
T40=T30*A43;


