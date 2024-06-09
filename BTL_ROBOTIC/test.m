% 
% x_0 = 800;
% y_0 = 0;
% z_0 = 0;
% x_1 = 650;
% y_1 = -200;
% z_1 = 0;
% x_2 = 100;
% y_2 = 600;
% z_2 = 0;
% 
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
% alpha=acos((2*R*R-(x_2-O(1)-R)^2-(y_2-O(2))^2)/(2*R*R));
% if y_1 > y_0
%     a=1;
% else 
%     a=-1;
% end
% if alpha>alpha_1
%     alpha_1=alpha;
% end
% if (y_0 < O(2))&&(y_2<O(2))
%     alpha_0 = -alpha_0;
%     alpha_1 = -alpha_1;
% end
% if (y_0 < O(2))&&(y_2>= O(2))
%     alpha_0 = -alpha_0;
%     alpha_1 =alpha_1-2*pi;
% end
% if (y_0 >= O(2))&&(y_2< O(2))
%     alpha_1 = -alpha_1;
% end
% if (y_0 >= O(2))&&(y_2>= O(2))
%     alpha_1 =alpha_1-2*pi;
% end
% dentaalpha = abs(alpha_0-alpha_1);
% 
% points = [];
% for k=1:30 
%     x = O(1)+R*cos(alpha_0+(k/30)*dentaalpha*a);
%     y = O(2)+R*sin(alpha_0+(k/30)*dentaalpha*a);
%     z = -x*n(1)/n(3)-y*n(2)/n(3)+d/n(3);
%     point = [x, y, z];  % Tạo một điểm mới có tọa độ (x, y, z)
%     points = [points; point];  % Thêm điểm vào mảng points
% end
% 
% figure;
% hold on;
% 
% plot3(points(:,1), points(:,2), points(:,3), 'ro', 'MarkerSize', 1);
% scatter3(O(1), O(2), O(3), 'r', 'filled');  % Vẽ tâm 
% scatter3(x_0, y_0, z_0, 'r');  
% scatter3(x_1, y_1, z_1, 'g'); 
% scatter3(x_2, y_2, z_2, 'm');  
% axis equal;
% title('Khối cầu');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% grid on;
% view(3


% % Tạo một đối tượng InputParameter
% inp = struct();
% inp.degrees_of_freedom = 3;
% inp.current_position = [0.0, -2.0, 0.0];
% inp.current_velocity = [0.0, 0.0, 0.0];
% inp.current_acceleration = [0.0, 0.0, 0.0];
% inp.target_position = [1.0, -3.0, 3.0];
% inp.target_velocity = [0.0, 0.0, 0.0];
% inp.target_acceleration = [0.0, 0.0, 0.0];
% inp.max_velocity = [100.0, 100.0, 100.0];
% inp.max_acceleration = [10.0, 10.0, 10.0];
% inp.max_jerk = [10.0, 10.0, 10.0];
% 
% % Tạo đối tượng Ruckig
% otg = Ruckig(inp.degrees_of_freedom, 0.001);
% 
% % Hàm walk_through_trajectory tương đương
% out_list = [];
% out = struct();
% out.new_position = inp.current_position;
% out.new_velocity = inp.current_velocity;
% out.new_acceleration = inp.current_acceleration;
% res = 1;
% while res == 1
%     res = otg.update(inp, out);
%     inp.current_position = out.new_position;
%     inp.current_velocity = out.new_velocity;
%     inp.current_acceleration = out.new_acceleration;
%     out_list = [out_list, copy(out)];
% end

% disp(length(out_list));
% disp(['Calculation duration: ', num2str(out_list(1).calculation_duration), ' [µs]']);
% disp(['Trajectory duration: ', num2str(out_list(1).trajectory.duration), ' [s]']);

% % py.runfile('E:\python_rickig\test.py');
% % 
% % % a_py = py.workspace.importFrom('test', 'a');
% py_code = fileread('E:\python_rickig\test.py');
% py.exec(py_code);
% %disp(a_py);
% system('E:\python_rickig\test.py');
% data = pyrunfile("test.py", "a");
% disp(data);
% system('E:\python_rickig\test.py > output.txt');
% pe = pyenv
% pathto= fileparts(which('hhhh.py'));
% if count(py.sys.path, pathto) == 0 
%   insert(py.sys.path, int32(0), pathto);
% end
% 
vel = 120;
accel = 40;
jerk = 80;
maxVel = [vel*pi/180, vel*pi/180, vel*pi/180, vel*pi/180, vel*pi/180, vel*pi/180];
maxAccel = [accel*pi/180, accel*pi/180, accel*pi/180, accel*pi/180, accel*pi/180, accel*pi/180];
maxJerk = [jerk*pi/180, jerk*pi/180, jerk*pi/180, jerk*pi/180, jerk*pi/180, jerk*pi/180];
targetPoshome=[0,0,0,0,0,0];
targetPos=[0,1,1,0,0,0];

% Convert numeric array to cell array of character vectors
str_maxvel = num2cell(maxVel);
str_maxaccel = num2cell(maxAccel);
str_maxjerk = num2cell(maxJerk);
% Convert cell array of character vectors to string array
str_maxvel = string(str_maxvel);
str_maxaccel = string(str_maxaccel);
str_maxjerk = string(str_maxjerk);
% Join the elements with a space delimiter
maxvel_inv = strjoin(str_maxvel, ' ');
maxaccel_inv = strjoin(str_maxaccel, ' ');
maxjerk_inv = strjoin(str_maxjerk, ' ');


% Đầu vào từ MATLAB
c_pos_m = '0 0 0 0 0 0';
t_pos_m = '1 1 1 0 0 0';
max_vel_m = maxvel_inv;
max_acc_m = maxaccel_inv;
max_jerk_m = maxjerk_inv;
step_time_m = '0.1';


% Tạo lệnh gọi Python
% python_cmd = ['python E:\BTL_ROBOTIC\ruckig_pos.py "', c_pos_m, '" "', t_pos_m, '" "', max_vel_m, '" "', max_acc_m, '" "', max_jerk_m, '" "', step_time_m, '"'];
python_cmd = sprintf('python E:\\BTL_ROBOTIC\\ruckig_pos.py "%s" "%s" "%s" "%s" "%s" "%s" "%s"', c_pos_m, t_pos_m, max_vel_m, max_acc_m, max_jerk_m, step_time_m);
% Gọi chương trình Python từ MATLAB
[status,pos] = system(python_cmd);
% Xóa ký tự '[' và ']' để tạo thành một chuỗi con
data_str = strrep(pos, '[', '');
data_str = strrep(data_str, ']', '');
% Tách chuỗi thành từng hàng
rows = strsplit(data_str, ', ');
% Đánh giá từng hàng thành mảng
data = cellfun(@eval, rows, 'UniformOutput', false);
% Chuyển đổi cell array thành ma trận
data = cell2mat(data);
% Reshape ma trận thành ma trận 6 cột
data = reshape(data, 6, []).';

% In kết quả
disp(data(1,1));
