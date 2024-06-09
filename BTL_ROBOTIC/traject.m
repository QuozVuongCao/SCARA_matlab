% 
% a1 = 450; a2= 400;
% x_0 = 0;
% y_0 = -700;
% z_0 = 0;
% yaw_0= 0;
% yaw_1= 1;
% 
% x_1 = 500;
% y_1 = -400;
% z_1 = 0;
% 
% x_2 = 10;
% y_2 = 700;
% z_2 =0;
% 
% %%%%%%%%%%%%%%
% AC = [x_2-x_0;y_2-y_0;z_2-z_0];
% AB = [x_1-x_0;y_1-y_0;z_1-z_0];
% n = cross(AC,AB);
% d = n'*[x_0;y_0;z_0];
% % Tam duong tron
% M = [n';...
%     2*(x_1-x_0) 2*(y_1-y_0) 2*(z_1-z_0);...
%     2*(x_2-x_0) 2*(y_2-y_0) 2*(z_2-z_0)];
% N = [d;x_1^2+y_1^2+z_1^2-x_0^2-y_0^2-z_0^2;x_2^2+y_2^2+z_2^2-x_0^2-y_0^2-z_0^2];
% O = M\N;
% R= sqrt((x_1-O(1))^2+(y_1-O(2))^2 );
% 
% %%%%%%%%%%%%
% alpha_0=acos((2*R*R-(x_0-O(1)-R)^2-(y_0-O(2))^2)/(2*R*R));
% alpha_1=acos((2*R*R-(x_1-O(1)-R)^2-(y_1-O(2))^2)/(2*R*R));
% alpha_2=acos((2*R*R-(x_2-O(1)-R)^2-(y_2-O(2))^2)/(2*R*R));
% if (y_0<O(2))
%     alpha_0=2*pi-alpha_0;
% end
% if (y_1<O(2))
%     alpha_1=2*pi-alpha_1;
% end
% if (y_2<O(2))
%     alpha_2=2*pi-alpha_2;
% end
% alpha= alpha_2-alpha_0;
% if (alpha>0)
%     if (alpha_2<pi && alpha_1>=pi)
%     dau=-1;
%     dentaalpha=2*pi-alpha;
%     else 
%        dau=1;
%         dentaalpha=alpha;
%       
% 
%     end
% 
% else
%     if (alpha_2<pi && alpha_1<=pi)
%     dau=1;
%     dentaalpha=alpha;
%     else
%         dau=1;
%     dentaalpha=2*pi+alpha;
%     end
% end
% 
% disp(alpha_0);
% disp(alpha_1);
% disp(alpha_2);
% disp(dentaalpha);
% 
% qMax    = R*abs(dentaalpha);
% 
% aMax = 100;
% % vMax = sqrt(qMax*aMax);
% vMax= 200;
% jerk_max = 100;
% 
% va=aMax^2/jerk_max;
% sa= 2*aMax^3/(jerk_max^2);
% if(vMax*jerk_max <aMax^2)
%     sv=2*vMax*sqrt(vMax/jerk_max);
% else
%     sv= vMax*(vMax/aMax+aMax/jerk_max);
% end
% 
% 
% if ((vMax>=va && qMax<=sa)|| (vMax<va && qMax<sa && qMax<sv))
%     tj=(qMax/(2*jerk_max))^(1/3);
%     ta=tj;
%     tv=2*tj;
% end
% if ((vMax<=va && qMax>=sa)||(vMax<=va &&qMax<=sa&&qMax>=sv))
%     tj=sqrt(vMax/jerk_max);
%     ta=tj;
%     tv=qMax/vMax;
% end
% if(vMax>=va && qMax>sa && qMax>=sv )
%     tj=aMax/jerk_max;
%     ta=vMax/aMax;
%     tv=qMax/vMax;
% end
% if (vMax>=va && qMax>sa && qMax<sv)
% %     tj=aMax/jerk_max;
% %     ta= 0.5*(sqrt((4*qMax*jerk_max^2+aMax^3)/(aMax*jerk_max^2)-aMax/jerk_max));
% %     tv= ta+tj;
%     tj=(qMax/(2*jerk_max))^(1/3);
%     ta=tj;
%     tv=2*tj;
% end
% 
% t_1= tj;
% t_2= ta;
% t_3= tj+ta;
% t_4= tv;
% t_5= tj+tv;
% t_6= tv+ta;
% tmax= tv+ta+tj;
% 
% 
% t       = 0:0.2:tmax;
% lengthT = length(t);
% a       = zeros(lengthT,1);
% v       = zeros(lengthT,1);
% q       = zeros(lengthT,1);
% Th_1=zeros(lengthT,1);
% Th_2=zeros(lengthT,1);
% d_3=zeros(lengthT,1);
% Th_4=zeros(lengthT,1);
% x_pre=x_0;
% y_pre=y_0;
% z_pre=z_0;
% yaw_pre=yaw_0;
% X=[];
% Y=[];
% Z=[];
% 
% for i = 1:1:lengthT
%     if(t(i)<t_1)
%         jerk(i)=jerk_max;
%         a(i) = jerk_max*t(i); 
%         a1=jerk_max*t_1;
%         v(i) = 0.5*jerk_max*t(i)^2;
%         v1= jerk_max*t_1^2/2;
%         q(i) = jerk_max*t(i)^3/6;
%         q1= jerk_max*t_1^3/6;
%     elseif (t(i)<t_2)
%         a1=jerk_max*t_1;
%         v1= jerk_max*t_1^2/2;
%         q1= jerk_max*t_1^3/6;
%         jerk(i)=0;
%         a(i) = a1;
%         a2=a1;
%         v(i) = v1+a1*(t(i)-t_1);
%         v2= v1+a1*(t_2-t_1);
%         q(i) = q1+v1*(t(i)-t_1)+a1*(t(i)-t_1)^2/2;
%         q2= q1+v1*(t_2-t_1)+a1*(t_2-t_1)^2/2;
%     elseif(t(i)<t_3)
%         a2=a1;
%         v2= v1+a1*(t_2-t_1);
%         q2= q1+v1*(t_2-t_1)+a1*(t_2-t_1)^2/2;
%         jerk(i)=-jerk_max;
%         a(i) =a2- jerk_max*(t(i)-t_2);
%         %a3= a2-jerk_max*(t_3-t_2);
%         a3=0;
%         v(i) =v2+a2*(t(i)-t_2)- jerk_max*(t(i)-t_2)^2/2;
%         v3= v2+a2*(t_3-t_2)-jerk_max*(t_3-t_2)^2/2;
%         q(i) = q2+v2*(t(i)-t_2)+a2*(t(i)-t_2)^2/2-jerk_max*(t(i)-t_2)^3/6;
%         q3= q2+v2*(t_3-t_2)+a2*(t_3-t_2)^2/2-jerk_max*(t_3-t_2)^3/6;
% 
%     elseif(t(i)<t_4)
%         v3= v2+a2*(t_3-t_2)-jerk_max*(t_3-t_2)^2/2;
%         q3= q2+v2*(t_3-t_2)+a2*(t_3-t_2)^2/2-jerk_max*(t_3-t_2)^3/6;
%         jerk(i)=0;
%         a(i) = 0;
%         a4=0;
%         v(i) = v3;
%         v4= v3;
%         q(i) = q3+v3*(t(i)-t_3);
%         q4= q3+v3*(t_4-t_3);
% 
%     elseif(t(i)<t_5)
%         v4= v3;
%         q4= q3+v3*(t_4-t_3);
%         jerk(i)=-jerk_max;
%         a(i) = -jerk_max*(t(i)-t_4);
%         a5= -jerk_max*(t_5-t_4);
%         v(i) = v4-jerk_max*(t(i)-t_4)^2/2;
%         v5= v4-jerk_max*(t_5-t_4)^2/2;
%         q(i) = q4+v4*(t(i)-t_4)-jerk_max*(t(i)-t_4)^3/6;
%         q5= q4+v4*(t_5-t_4)-jerk_max*(t_5-t_4)^3/6;
% 
%     elseif(t(i)<t_6)
%         v4= v3;
%         q4= q3+v3*(t_4-t_3);
%         a5= -jerk_max*(t_5-t_4);
%         v5= v4-jerk_max*(t_5-t_4)^2/2;
%         q5= q4+v4*(t_5-t_4)-jerk_max*(t_5-t_4)^3/6;
%         jerk(i)=0;
%         a(i) = a5;
%         a6= a5;% =-a1
%         v(i) = v5- aMax*(t(i)-t_5);
%         v6= v5-aMax*(t_6-t_5);
%         q(i) = q5+v5*(t(i)-t_5)+a5*(t(i)-t_5)^2/2;
%         q6= q5+v5*(t_6-t_5)+a5*(t_6-t_5)^2/2;
%     else
%         a6= a5;% =-a1
%         v6= v5-aMax*(t_6-t_5);
%         q6= q5+v5*(t_6-t_5)+a5*(t_6-t_5)^2/2;
%         jerk(i)=jerk_max;
%         a(i) = a6+jerk_max*(t(i)-t_6);
%         a7=0;
%         v(i) = v6+a6*(t(i)-t_6)+jerk_max*(t(i)-t_6)^2/2;
%         v7=0;
%         q(i) = q6+v6*(t(i)-t_6)+a6*(t(i)-t_6)^2/2+jerk_max*(t(i)-t_6)^3/6;
% 
%     end  
% end
% 
% 
% 
% for i = 1:1:lengthT
%     x = O(1)+R*cos(alpha_0+(q(i)/qMax)*dentaalpha*dau);
%     y = O(2)+R*sin(alpha_0+(q(i)/qMax)*dentaalpha*dau);
%     z = -x*n(1)/n(3)-y*n(2)/n(3)+d/n(3)-(q(i)/qMax)*(abs(z_0-z_1));
%     yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
%     X=[X, x];
%     Y=[Y, y];
%     Z=[Z, z];
%     [Theta_1, Theta_2, d3, Theta_4] = Inverse(x, y,z, yaw);
%     Th_1(i)=Theta_1;
%     Th_2(i)=Theta_2;
%     Th_4(i)=Theta_4;
%     d_3(i)=d3;
% 
%     v_end=[((x-x_pre)/0.2);
%            ((y-y_pre)/0.2);
%            ((z-z_pre)/0.2);
%            ((yaw-yaw_pre)/0.2)];
% 
%     Jacobian_Matrix=[- 450*sin(Theta_1) - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), 0, 0;
%   450*cos(Theta_1) + 400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2),   400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2), 0, 0;
%                                                                                  0,                                                               0, 1, 0;
%                                                                                   1                                        1                       0   1];
%     v_joint=(Jacobian_Matrix)\v_end;
%     v_th1(i)=v_joint(1,1);
%     v_th2(i)=v_joint(2,1);
%     v_d3(i) =v_joint(3,1);
%     v_th4(i)=v_joint(4,1);
%     x_pre=x;
%     y_pre=y;
%     z_pre=z;
%     yaw_pre=yaw;
% 
% 
% 
% end
% 
% figure(1);
% scatter3(X, Y, Z, 'r', 'filled');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% grid on;

x=850;
y=0;
z=0;
yaw=1;
[Theta_1, Theta_2, d3, Theta_4] = Inverse(x, y,z, yaw);

 Jacobian_Matrix=[- 450*sin(Theta_1) - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), 0, 0;
  450*cos(Theta_1) + 400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2),   400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2), 0, 0;
                                                                                 0,                                                               0, 1, 0;
                                                                                  1                                        1                       0   1];
determinant = det(Jacobian_Matrix);    
disp(determinant);


