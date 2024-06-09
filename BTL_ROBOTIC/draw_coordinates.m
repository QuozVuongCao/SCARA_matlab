function draw_coordinates(x, y, z, mt)  
%x_axis
plot3([x (100*mt(1,1)+x)],[y (100*mt(2,1)+y)],[z (100*mt(3,1)+z)],'r','linewidth',2);
text((130*mt(1,1)+x),(130*mt(2,1)+y),(130*mt(3,1)+z),'x','HorizontalAlignment','right','FontSize',11);
%y_axis
plot3([x (100*mt(1,2)+x)],[y (100*mt(2,2)+y)],[z (100*mt(3,2)+z)],'g','linewidth',2);
text((130*mt(1,2)+x),(130*mt(2,2)+y),(130*mt(3,2)+z),'y','HorizontalAlignment','right','FontSize',11);
%z_axis
plot3([x (100*mt(1,3)+x)],[y (100*mt(2,3)+y)],[z (100*mt(3,3)+z)],'b','linewidth',2);
text((130*mt(1,3)+x),(130*mt(2,3)+y),(130*mt(3,3)+z),'z','HorizontalAlignment','right','FontSize',11);