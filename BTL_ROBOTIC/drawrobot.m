function drawrobot(theta_1,theta_2,d_3,theta_4,opa,handles)
% theta_1=0;
% theta_2=0;
% theta_4=0;
% d_3=0;
% opa=0.5;
%%%%%
axes(handles.axes1);
cla(handles.axes1);
[T10 T20 T30 T40] = forward(theta_1, theta_2, d_3, theta_4);

px1=T10(1,4);    
py1=T10(2,4);   
pz1=T10(3,4);  

px2=T20(1,4);    
py2=T20(2,4);    
pz2=T20(3,4); 

px3=T30(1,4);   
py3=T30(2,4);    
pz3=T30(3,4);    

px4=T40(1,4);    
py4=T40(2,4);    
pz4=T40(3,4);

th1 = theta_1;
th2 = theta_2 + th1;
th4 = th2 + theta_4;

% Plot Coordinates
grid on
T00 = [  1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
base = [ 0 ; 0 ; 0 ; 1]; 
x0 = base(1,1); y0 = base(2,1); z0 =base(3,1);
joint1 = T10*base;
joint2 = T20*base;
joint3 = T30*base;
joint4 = T40*base;
%END EFFECTOR
x1= joint1(1,1); y1= joint1(2,1); z1= joint1(3,1);
x2= joint2(1,1); y2= joint2(2,1); z2= joint2(3,1);
x3= joint3(1,1); y3= joint3(2,1); z3= joint3(3,1);
x4= joint4(1,1); y4= joint4(2,1); z4= joint4(3,1);

if handles.checkbox_c0.Value
    draw_coordinates(x0,y0,z0,T00);
    hold on
end
if handles.checkbox_c1.Value
    draw_coordinates(x1,y1,z1,T10);
    hold on
end
if handles.checkbox_c2.Value
    draw_coordinates(x2,y2,z2+80,T20);
    hold on
end
if handles.checkbox_c3.Value
    draw_coordinates(x3,y3,z3,T30);
    hold on
end
if handles.checkbox_c4.Value
    draw_coordinates(x4,y4,z4,T40);
    hold on
end
if handles.checkbox_workspace.Value
    [x,z]=meshgrid(-850:5:850,0:400);
    y=sqrt(850.^2-x.^2);
    s = surf(x,y,z,'FaceAlpha',0.5);
    y1=-sqrt(850.^2-x.^2);
    s1 = surf(x,y1,z,'FaceAlpha',0.5);
    s.EdgeColor = 'none';
    s1.EdgeColor = 'none';

    [x,z]=meshgrid(-260:5:260,0:400);
    y=sqrt(260.^2-x.^2);
    s = surf(x,y,z,'FaceAlpha',0.5);
    y1=-sqrt(260.^2-x.^2);
    s1 = surf(x,y1,z,'FaceAlpha',0.5);
    s.EdgeColor = 'none';
    s1.EdgeColor = 'none';
end


% base
grid on
%opa= str2double(get(handles.eboxOpacity,'String'));
%opa=0.2;
r1 = 170;
[X1 Y1 Z1] = cylinder(r1);
Z1 = Z1*363;
hold on
surf(X1, Y1, Z1,'FaceColor',[0, 100, 173]/255, 'Facealpha',opa)
fill3(X1', Y1', Z1', [0, 100, 173]/255, 'Facealpha',opa)

%Joint 0
r2 = 150 ; %158
[X2 Y2 Z2] = cylinder (r2);
Z2 = Z2*80;
surf(X2, Y2, Z2 + pz1, 'FaceColor',[0, 200, 50]/255,'EdgeColor','none','Facealpha',opa)
fill3(X2', Y2', Z2' + pz1, [0, 200, 50]/255,'Facealpha',opa)

% Joint 1
r3 = 150;
[X3 Y3 Z3] = cylinder(r3);
Z3 = Z3*80;

surf(X3+px1, Y3+py1, Z3+pz1, 'FaceColor',[0, 200, 50]/255,'EdgeColor','none','Facealpha',opa)
fill3(X3'+px1, Y3'+py1, Z3'+pz1,[0, 200, 50]/255,'Facealpha',opa)

r4 = 150;

[X4 Y4 Z4] = cylinder(r4);
Z4 = Z4*150;
surf(X4+px1, Y4+py1, Z4+pz1+80, 'FaceColor', [0, 150, 71]/255,'EdgeColor','none','Facealpha',opa)
fill3(X4'+px1, Y4'+py1, Z4'+pz1+80, [0, 150, 71]/255,'Facealpha',opa)

% Joint 2
r5 = 100;
[X5 Y5 Z5] = cylinder(r5);
Z5 = Z5*150;
surf(X5+px2, Y5+py2, Z5+pz2+80, 'FaceColor',[0, 150, 71]/255,'EdgeColor','none','Facealpha',opa)
fill3(X5'+px2, Y5'+py2, Z5'+pz2+80,[0, 150, 71]/255,'Facealpha',opa)

%Joint 3
r6 = 30;
[X6 Y6 Z6] = cylinder(r6);
Z6 = Z6*500;
surf(X6+px3, Y6+py3, Z6+pz4, 'FaceColor', [229, 100, 0]/255,'EdgeColor','none','Facealpha',opa)
fill3(X6'+px3, Y6'+py3, Z6'+pz4, [229, 100, 0]/255,'Facealpha',opa)

% r7 = 1000;
% [X7 Y7 Z7] = cylinder(r7);
% Z7 = Z7*200;
% surf(X7+px3, Y7+py3, Z7+pz3, 'FaceColor', 'yellow','EdgeColor','none')
% fill3(X7'+px3, Y7'+py3, Z7'+pz3, 'yellow')

%Link 1
Link1_X = [r2*cos(th1+pi/2), -r2*cos(th1+pi/2), px1-r3*cos(th1+pi/2), px1+r3*cos(th1+pi/2); 
            r2*cos(th1+pi/2), -r2*cos(th1+pi/2), px1-r3*cos(th1+pi/2), px1+r3*cos(th1+pi/2);
            r2*cos(th1+pi/2), r2*cos(th1+pi/2), px1+r4*cos(th1+pi/2), px1+r4*cos(th1+pi/2);
            -r2*cos(th1+pi/2), -r2*cos(th1+pi/2), px1-r4*cos(th1+pi/2), px1-r4*cos(th1+pi/2)];
Link1_Y = [r2*sin(th1+pi/2), -r2*sin(th1+pi/2), py1-r3*sin(th1+pi/2), py1+r3*sin(th1+pi/2); 
            r2*sin(th1+pi/2), -r2*sin(th1+pi/2), py1-r3*sin(th1+pi/2), py1+r3*sin(th1+pi/2);
            r2*sin(th1+pi/2), r2*sin(th1+pi/2), py1+r4*sin(th1+pi/2), py1+r4*sin(th1+pi/2);
            -r2*sin(th1+pi/2), -r2*sin(th1+pi/2), py1-r4*sin(th1+pi/2), py1-r4*sin(th1+pi/2)];
Link1_Z = [pz1+80,        pz1+80,        pz1+80,        pz1+80; 
            pz1+80,    pz1+80,    pz1+80,    pz1+80;
            pz1,     pz1+80,    pz1+80,    pz1; 
             pz1,     pz1+80,    pz1+80,    pz1];
fill3(Link1_X', Link1_Y', Link1_Z',[0, 200, 50]/255 ,'Facealpha',opa)



% Link2
Link2_X = [ px1+r4*cos(th2+pi/2), px1-r4*cos(th2+pi/2), px2-r5*cos(th2+pi/2), px2+r5*cos(th2+pi/2); 
             px1+r4*cos(th2+pi/2), px1-r4*cos(th2+pi/2), px2-r5*cos(th2+pi/2), px2+r5*cos(th2+pi/2);
             px1+r4*cos(th2+pi/2), px1+r4*cos(th2+pi/2), px2+r5*cos(th2+pi/2), px2+r5*cos(th2+pi/2);
            px1-r4*cos(th2+pi/2), px1-r4*cos(th2+pi/2), px2-r5*cos(th2+pi/2), px2-r5*cos(th2+pi/2)];
Link2_Y = [py1+r4*sin(th2+pi/2), py1-r4*sin(th2+pi/2), py2-r5*sin(th2+pi/2), py2+r5*sin(th2+pi/2); 
             py1+r4*sin(th2+pi/2), py1-r4*sin(th2+pi/2), py2-r5*sin(th2+pi/2), py2+r5*sin(th2+pi/2);
             py1+r4*sin(th2+pi/2), py1+r4*sin(th2+pi/2), py2+r5*sin(th2+pi/2), py2+r5*sin(th2+pi/2);
            py1-124*sin(th2+pi/2), py1-r4*sin(th2+pi/2), py2-r5*sin(th2+pi/2), py2-r5*sin(th2+pi/2)];
Link2_Z = [pz2+80,         pz2+80,          pz2+80,          pz2+80; 
             pz2 + 150+80,   pz2 + 150+80,    pz2 + 150+80,    pz2 + 150+80;
             pz2+150+80,     pz2 + 150+80,    pz2 + 150+80,    pz2+150+80; 
             pz2+80,     pz2+150+80,    pz2+150+80,    pz2+80];
fill3(Link2_X', Link2_Y', Link2_Z',[0, 150, 71]/255,'Facealpha',opa)

%end-effector
r9 = 20;
[X9 Y9 Z9] = cylinder(r9);
Z9 = Z9*30;
surf(X9 + px4 + 30*cos(th4), Y9 + py4 + 30*sin(th4), Z9+pz4-30, 'FaceColor',[166, 61, 64]/255,'EdgeColor','none')
surf(X9 + px4 - 30*cos(th4), Y9 + py4 - 30*sin(th4), Z9+pz4-30, 'FaceColor',[166, 61, 64]/255,'EdgeColor','none')
fill3(X9' + px4 + 30*cos(th4), Y9' + py4 + 30*sin(th4), -Z9'+pz4, [166, 61, 64]/255)
fill3(X9' + px4 - 30*cos(th4), Y9' + py4 - 30*sin(th4), -Z9'+pz4, [166, 61, 64]/255)

%workspace(opa);

axis([-1000 1000 -1000 1000 0 1000]);
xlabel('x');
ylabel('y');
zlabel('z');
rotate3d on