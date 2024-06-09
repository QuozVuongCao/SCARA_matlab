
syms theta_1;
syms theta_2;
syms theta_4;
syms d_3;

T10 = [cos(theta_1), -sin(theta_1), 0, 450*cos(theta_1);
sin(theta_1),  cos(theta_1), 0, 450*sin(theta_1);
           0,             0, 1,              363;
           0,             0, 0,                1];
 
 
T20 =[cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2), - cos(theta_1)*sin(theta_2) - cos(theta_2)*sin(theta_1), 0, 450*cos(theta_1) + 400*cos(theta_1)*cos(theta_2) - 400*sin(theta_1)*sin(theta_2);
cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1),   cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2), 0, 450*sin(theta_1) + 400*cos(theta_1)*sin(theta_2) + 400*cos(theta_2)*sin(theta_1);
                                                    0,                                                       0, 1,                                                                              363;
                                                    0,                                                       0, 0,                                                                                1];
 
 
T30 =[cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2), - cos(theta_1)*sin(theta_2) - cos(theta_2)*sin(theta_1), 0, 450*cos(theta_1) + 400*cos(theta_1)*cos(theta_2) - 400*sin(theta_1)*sin(theta_2);
cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1),   cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2), 0, 450*sin(theta_1) + 400*cos(theta_1)*sin(theta_2) + 400*cos(theta_2)*sin(theta_1);
                                                    0,                                                       0, 1,                                                                        d_3 + 363;
                                                    0,                                                       0, 0,                                                                                1];
 
 
T40 =[cos(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)) - sin(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)), cos(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) + sin(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)),  0, 450*cos(theta_1) + 400*cos(theta_1)*cos(theta_2) - 400*sin(theta_1)*sin(theta_2);
cos(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) + sin(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)), sin(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) - cos(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)),  0, 450*sin(theta_1) + 400*cos(theta_1)*sin(theta_2) + 400*cos(theta_2)*sin(theta_1);
                                                                                                                                          0,                                                                                                                                           0, -1,                                                                              d_3;
                                                                                                                                          0,                                                                                                                                           0,  0,                                                                                1];
 
 
z0= [0;0;1];
p0= [0;0;0];
z1= [T10(1,3);T10(2,3);T10(3,3)];
p1= [T10(1,4);T10(2,4);T10(3,4)];
z2= [T20(1,3);T20(2,3);T20(3,3)];
p2= [T20(1,4);T20(2,4);T20(3,4)];
z3= [T30(1,3);T30(2,3);T30(3,3)];
p3= [T30(1,4);T30(2,4);T30(3,4)];
z4= [T40(1,3);T40(2,3);T40(3,3)];
p4= [T40(1,4);T40(2,4);T40(3,4)];

p_40= p4-p0;
p_41= p4-p1;
p_42= p4-p2;
p_43= p4-p3;
jacobian1= [xmatran(z0,p_40), xmatran(z1,p_41), xmatran(z2,p_42), xmatran(z3,p_43);
            z0,z1,z2,z3]
rank_A = rank(jacobian1);
disp(rank_A);



function [o3x1] =xmatran(u,p);
    o3x1=[(-u(3)*p(2)+u(2)*p(3)); u(3)*p(1)-u(1)*p(3); -u(2)*p(1)+u(1)*p(2)];
end
