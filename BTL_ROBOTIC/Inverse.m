function [the_1, the_2, d3, the_4] = Inverse(x, y,z, yaw)
        a1 = 450; a2= 400;
        x0 = 0; y0 = 0; z0 = 0;

%     a_1=450;
%     a_2=400;
%     d_1=363;
%     d_4=-330; %-420
    
        c2 = ((x^2 +y^2 -a1^2 -a2^2)/(2*a1*a2));
        s2 = sqrt(1 - c2^2);
%         if (handles.flagUp.Value)
%             s2 = -s2;
%         end
        the_2 = atan2(s2,c2);

%         the_2 = acos((x^2 +y^2 -a1^2 -a2^2)/(2*a1*a2));
%         if (y>0)
%                the_2 = -the_2;
%         end
        s1=((a1+a2*cos(the_2))*y-a2*sin(the_2)*x);
        c1= ((a1 + a2*cos(the_2))*x+(a2*sin(the_2))*y);
        %the_1 = atan(((a1+a2*cos(the_2))*y-a2*sin(the_2)*x)/((a1 + a2*cos(the_2))*x+(a2*sin(the_2))*y));
        the_1 = atan2(s1,c1);       
        the_4= yaw - the_1- the_2;
        if (the_4 > pi)
            the_4 = the_4 - 2*pi;
        end
        if (the_4 < -pi)
            the_4 = the_4 + 2*pi;
        end
        d3=z-z0;   
end 

% T40 =
%  
% [ cos(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)) - sin(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)), cos(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) + sin(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)),  0, 450*cos(theta_1) + 400*cos(theta_1)*cos(theta_2) - 400*sin(theta_1)*sin(theta_2)]
% [ cos(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) + sin(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)), sin(theta_4)*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) - cos(theta_4)*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)),  0, 450*sin(theta_1) + 400*cos(theta_1)*sin(theta_2) + 400*cos(theta_2)*sin(theta_1)]
% [                                                                                                                                           0,                                                                                                                                           0, -1,                                                                         d_3 - 57]
% [                                                                                                                                           0,                                                                                                                                           0,  