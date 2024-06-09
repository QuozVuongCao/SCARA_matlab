clear
close all

x1= 800;
y1=10;
z1= 0;
yaw_1= 0;
x2= 600;
y2= -600;
z2= 100;
yaw_2=3.14;
a1 = 450; a2= 400;
[the_1, the_2, d3, the_4] = Inverse(x1, y1,z1, yaw_1);
[the_1_m, the_2_m, d3_m, the_4_m] = Inverse(x2, y2,z2, yaw_2);

qMax =500;
vMax = 200;
aMax = 100;
jerk_max = 100;

va=aMax^2/jerk_max;
sa= 2*aMax^3/(jerk_max^2);
if(vMax*jerk_max <aMax^2)
    sv=2*vMax*sqrt(vMax/jerk_max);
else
    sv= vMax*(vMax/aMax+aMax/jerk_max);
end

if ((vMax>=va && qMax<=sa)|| (vMax<va && qMax<sa && qMax<sv))
    tj=(qMax/(2*jerk_max))^(1/3);
    ta=tj;
    tv=2*tj;
end
if ((vMax<=va && qMax>=sa)||(vMax<=va &&qMax<=sa&&qMax>=sv))
    tj=sqrt(vMax/jerk_max);
    ta=tj;
    tv=qMax/vMax;
end
if(vMax>=va && qMax>sa && qMax>=sv )
    tj=aMax/jerk_max;
    ta=vMax/aMax;
    tv=qMax/vMax;
end
if (vMax>=va && qMax>sa && qMax<sv)
%     tj=aMax/jerk_max;
%     ta= 0.5*(sqrt((4*qMax*jerk_max^2+aMax^3)/(aMax*jerk_max^2)-aMax/jerk_max));
%     tv= ta+tj;
    tj=(qMax/(2*jerk_max))^(1/3);
    ta=tj;
    tv=2*tj;
end

t_1= tj;
t_2= ta;
t_3= tj+ta;
t_4= tv;
t_5= tj+tv;
t_6= tv+ta;
tmax= tv+ta+tj;

t       = 0:0.01:tmax;
lengthT = length(t);
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
jerk= zeros(lengthT,1);


for i = 1:1:lengthT
    if(t(i)<t_1)
        jerk(i)=jerk_max;
        a(i) = jerk_max*t(i); 
        a1=jerk_max*t_1;
        v(i) = 0.5*jerk_max*t(i)^2;
        v1= jerk_max*t_1^2/2;
        q(i) = jerk_max*t(i)^3/6;
        q1= jerk_max*t_1^3/6;
    elseif (t(i)<t_2)
        a1=jerk_max*t_1;
        v1= jerk_max*t_1^2/2;
        q1= jerk_max*t_1^3/6;
        jerk(i)=0;
        a(i) = a1;
        a2=a1;
        v(i) = v1+a1*(t(i)-t_1);
        v2= v1+a1*(t_2-t_1);
        q(i) = q1+v1*(t(i)-t_1)+a1*(t(i)-t_1)^2/2;
        q2= q1+v1*(t_2-t_1)+a1*(t_2-t_1)^2/2;
    elseif(t(i)<t_3)
        a2=a1;
        v2= v1+a1*(t_2-t_1);
        q2= q1+v1*(t_2-t_1)+a1*(t_2-t_1)^2/2;
        jerk(i)=-jerk_max;
        a(i) =a2- jerk_max*(t(i)-t_2);
        %a3= a2-jerk_max*(t_3-t_2);
        a3=0;
        v(i) =v2+a2*(t(i)-t_2)- jerk_max*(t(i)-t_2)^2/2;
        v3= v2+a2*(t_3-t_2)-jerk_max*(t_3-t_2)^2/2;
        q(i) = q2+v2*(t(i)-t_2)+a2*(t(i)-t_2)^2/2-jerk_max*(t(i)-t_2)^3/6;
        q3= q2+v2*(t_3-t_2)+a2*(t_3-t_2)^2/2-jerk_max*(t_3-t_2)^3/6;

    elseif(t(i)<t_4)
        v3= v2+a2*(t_3-t_2)-jerk_max*(t_3-t_2)^2/2;
        q3= q2+v2*(t_3-t_2)+a2*(t_3-t_2)^2/2-jerk_max*(t_3-t_2)^3/6;
        jerk(i)=0;
        a(i) = 0;
        a4=0;
        v(i) = v3;
        v4= v3;
        q(i) = q3+v3*(t(i)-t_3);
        q4= q3+v3*(t_4-t_3);

    elseif(t(i)<t_5)
        v4= v3;
        q4= q3+v3*(t_4-t_3);
        jerk(i)=-jerk_max;
        a(i) = -jerk_max*(t(i)-t_4);
        a5= -jerk_max*(t_5-t_4);
        v(i) = v4-jerk_max*(t(i)-t_4)^2/2;
        v5= v4-jerk_max*(t_5-t_4)^2/2;
        q(i) = q4+v4*(t(i)-t_4)-jerk_max*(t(i)-t_4)^3/6;
        q5= q4+v4*(t_5-t_4)-jerk_max*(t_5-t_4)^3/6;

    elseif(t(i)<t_6)
        v4= v3;
        q4= q3+v3*(t_4-t_3);
        a5= -jerk_max*(t_5-t_4);
        v5= v4-jerk_max*(t_5-t_4)^2/2;
        q5= q4+v4*(t_5-t_4)-jerk_max*(t_5-t_4)^3/6;
        jerk(i)=0;
        a(i) = a5;
        a6= a5;% =-a1
        v(i) = v5- aMax*(t(i)-t_5);
        v6= v5-aMax*(t_6-t_5);
        q(i) = q5+v5*(t(i)-t_5)+a5*(t(i)-t_5)^2/2;
        q6= q5+v5*(t_6-t_5)+a5*(t_6-t_5)^2/2;
    else
        a6= a5;% =-a1
        v6= v5-aMax*(t_6-t_5);
        q6= q5+v5*(t_6-t_5)+a5*(t_6-t_5)^2/2;
        jerk(i)=jerk_max;
        a(i) = a6+jerk_max*(t(i)-t_6);
        a7=0;
        v(i) = v6+a6*(t(i)-t_6)+jerk_max*(t(i)-t_6)^2/2;
        v7=0;
        q(i) = q6+v6*(t(i)-t_6)+a6*(t(i)-t_6)^2/2+jerk_max*(t(i)-t_6)^3/6;

    end  
end

the1_pre= the_1;
the2_pre= the_2;
the4_pre= the_4;
d3_pre= d3;


for i = 1:1:lengthT
    theta1= the_1+(the_1_m-the_1)*q(i)/qMax;
    theta2= the_2+(the_2_m-the_2)*q(i)/qMax;
    theta4= the_4+(the_4_m-the_4)*q(i)/qMax;
    d_3= d3+(d3_m-d3)*q(i)/qMax;
    v_joint=[((theta1-the1_pre)/0.01);
               ((theta2-the2_pre)/0.01);
               ((d_3-d3_pre)/0.01);
               ((theta4-the4_pre)/0.01)];
%     Jacobian_Matrix=[   -a2*sin(theta1+theta2)-a1*sin(theta1)    -a2*sin(theta1+theta2)   0   0;
%                          a2*cos(theta1+theta2)+a1*cos(theta1)     a2*cos(theta1+theta2)   0   0;
%                          0                                        0                       1   0;
%                          1                                        1                       0   1];

    Jacobian_Matrix=[- 450*sin(theta1) - 400*cos(theta1)*sin(theta2) - 400*cos(theta2)*sin(theta1), - 400*cos(theta1)*sin(theta2) - 400*cos(theta2)*sin(theta1), 0, 0;
  450*cos(theta1) + 400*cos(theta1)*cos(theta2) - 400*sin(theta1)*sin(theta2),   400*cos(theta1)*cos(theta2) - 400*sin(theta1)*sin(theta2), 0, 0;
                                                                                 1,                                                               1, 1, 1;
                                                                                  1                                        1                       0   1];
 
    v_end=(Jacobian_Matrix)*v_joint;
    v_x(i)= v_end(1,1);
    v_y(i)= v_end(2,1);
    v_z(i)= v_end(3,1);
    v_yaw(i)= v_end(4,1);
    the1_pre= theta1;
    the2_pre= theta2;
    the4_pre= theta4;
    d3_pre= d_3;


end

%a = (gaussmf(t,[t1/8 t1/2]) - gaussmf(t,[t1/8 t2+t1/2]))*aMax;
x= cumtrapz(t,v_x);
y = cumtrapz(t,v_y);
z = cumtrapz(t,v_z);
yaww = cumtrapz(t,v_yaw);

% figure(1)
% subplot(4,1,1)
% hold on
% grid on
% plot(t,qm,'LineWidth',2);
% legend('q(t)');
% 
% subplot(4,1,2)
% hold on
% grid on
% plot(t,vm,'LineWidth',2);
% legend('v(t)');
% 
% subplot(4,1,3)
% hold on
% grid on
% plot(t,am,'LineWidth',2);
% legend('a(t)');
%     
% subplot(4,1,4)
% hold on
% grid on
% plot(t,jerk,'LineWidth',2);
% legend('jerk(t)');

pause(0.1);
figure(1)
subplot(4,1,1)
hold on
grid on
plot(t,q,'LineWidth',2);
legend('q(t)');
pause(0.1);
subplot(4,1,2)
hold on
grid on
plot(t,v,'LineWidth',2);
legend('v(t)');
pause(0.1);
subplot(4,1,3)
hold on
grid on
plot(t,a,'LineWidth',2);
legend('a(t)');
pause(0.1);
subplot(4,1,4)
hold on
grid on
plot(t,jerk,'LineWidth',2);
legend('jerk(t)');


figure(2)
subplot(4,2,1)
hold on
grid on
plot(t,v_x,'LineWidth',2);
legend('vx(t)');
pause(0.1);
subplot(4,2,2)
hold on
grid on
plot(t,v_y,'LineWidth',2);
legend('vy(t)');
pause(0.1);
subplot(4,2,3)
hold on
grid on
plot(t,v_z,'LineWidth',2);
legend('vz(t)');
pause(0.1);
subplot(4,2,4)
hold on
grid on
plot(t,v_yaw,'LineWidth',2);
legend('yaw(t)');



figure(3)
subplot(4,3,1)
hold on
grid on
plot(t,x,'LineWidth',2);
legend('x(t)');
pause(0.1);
subplot(4,3,2)
hold on
grid on
plot(t,y,'LineWidth',2);
legend('y(t)');
pause(0.1);
subplot(4,3,3)
hold on
grid on
plot(t,z,'LineWidth',2);
legend('z(t)');
pause(0.1);
subplot(4,3,4)
hold on
grid on
plot(t,yaww,'LineWidth',2);
legend('yaw(t)');
