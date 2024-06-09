
function varargout = robot_gui(varargin)
% ROBOT_GUI MATLAB code for robot_gui.fig
%      ROBOT_GUI, by itself, creates a new ROBOT_GUI or raises the existing
%      singleton*.
%
%      H = ROBOT_GUI returns the handle to a new ROBOT_GUI or the handle to
%      the existing singleton*.
%
%      ROBOT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOT_GUI.M with the given input arguments.
%
%      ROBOT_GUI('Property','Value',...) creates a new ROBOT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before robot_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to robot_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help robot_gui

% Last Modified by GUIDE v2.5 11-Dec-2023 11:23:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @robot_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @robot_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before robot_gui is made visible.
function robot_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to robot_gui (see VARARGIN)

% Choose default command line output for robot_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes robot_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = robot_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider_theta1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

theta1 = (pi/180)*(get(handles.slider_theta1,'Value'));
set(handles.edit_theta1,'String',theta1*180/pi);


% --- Executes during object creation, after setting all properties.
function slider_theta1_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider_theta2_Callback(hObject, eventdata, handles)

theta2 = (pi/180)*(get(handles.slider_theta2,'Value'));
set(handles.edit_theta2,'String',theta2*180/pi);

% --- Executes during object creation, after setting all properties.
function slider_theta2_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_d3_Callback(hObject, eventdata, handles)

d3 = get(handles.slider_d3,'Value');
set(handles.edit_d3,'String',d3);
    

% --- Executes during object creation, after setting all properties.
function slider_d3_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_theta4_Callback(hObject, eventdata, handles)

theta4 = (pi/180)*(get(handles.slider_theta4,'Value'));
set(handles.edit_theta4,'String',theta4*180/pi);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta4_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit1_Callback(hObject, eventdata, handles)

function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit3_Callback(hObject, eventdata, handles)

function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit4_Callback(hObject, eventdata, handles)

function edit4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit5_Callback(hObject, eventdata, handles)

function edit5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre; 

t1     = str2double(get(handles.edit_theta1,'String'));
t2     = str2double(get(handles.edit_theta2,'String'));
d3     = str2double(get(handles.edit_d3,'String'));
t4     = str2double(get(handles.edit_theta4,'String'));
set(handles.slider_theta1, 'value',t1);
set(handles.slider_theta2, 'value',t2);
set(handles.slider_theta4, 'value',t4);
set(handles.slider_d3, 'value',d3);

theta1 = (pi/180)*(get(handles.slider_theta1,'Value'));  
theta2 = (pi/180)*(get(handles.slider_theta2,'Value'));
theta4 = (pi/180)*(get(handles.slider_theta4,'Value'));    
d3 = get(handles.slider_d3,'Value');
    
[T10 T20 T30 T40] = forward(theta1, theta2, d3, theta4);
Px=T40(1,4);
Py=T40(2,4);
Pz=T40(3,4);
set(handles.edit_posx,'string',num2str(Px));
set(handles.edit_posy,'string',num2str(Py));
set(handles.edit_posz,'string',num2str(Pz));
%ROLL-PITCH-YAW
roll  = atan2(T40(2,3),T40(3,3));  %roll
pitch  = asin(-T40(3,1)); %pitch
yaw  = atan2(T40(2,1),T40(1,1));  %yaw
set(handles.edit_roll,'String',roll);
set(handles.edit_pitch,'String',pitch);
set(handles.edit_yaw,'String',yaw);

% alpha= atan2(-Z2, Z3)
% beta= asin(Z1)
% gamma = atan2(-Y1,X1)

opa=opa_pre;
%drawrobot(theta1,theta2,d3,theta4,opa);
the1= theta1_pre;
the2= theta1_pre;
the4= theta1_pre;
set(handles.forward, 'Enable', 'off');
numbers = [abs(theta1-the1)/0.05, abs(theta2-the2)/0.05, abs(theta4-the4)/0.05,10];
N = max(numbers); 

for k=1:N 
     Theta_1_temp = theta1_pre + (theta1-theta1_pre)*(k)/N;
     Theta_2_temp = theta2_pre + (theta2-theta2_pre)*(k)/N;
     d_3_temp = d3_pre+ (d3-d3_pre)*(k)/N;
     Theta_4_temp = theta4_pre+ (theta4-theta4_pre)*(k)/N;
     
     drawrobot(Theta_1_temp,Theta_2_temp,d_3_temp, Theta_4_temp,opa,handles);
     pause(0.2);
end
set(handles.forward, 'Enable', 'on');
theta1_pre = theta1;
theta2_pre = theta2;
d3_pre=d3;
theta4_pre = theta4; 


% --- Executes on button press in inverse.
function inverse_Callback(hObject, eventdata, handles)
% hObject    handle to inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;

x = str2double(handles.edit_posx.String);
y = str2double(handles.edit_posy.String);
z = str2double(handles.edit_posz.String);
yaw= str2double(handles.edit_yaw.String);

% [theta1, theta2,d3,theta4]= Inverse(x,y,z,yaw);
try
    [theta1, theta2,d3,theta4]= Inverse(x,y,z,yaw);
catch exception
    msg = exception.message;
    msgbox(msg, 'Error');
end


set(handles.slider_theta1, 'value',(theta1*180/pi));
set(handles.slider_theta2, 'value',(theta2*180/pi));
set(handles.slider_theta4, 'value',(theta4*180/pi));
set(handles.slider_d3, 'value',d3);

set(handles.edit_theta1,'string',num2str(theta1*180/pi));
set(handles.edit_theta2,'string',num2str(theta2*180/pi));
set(handles.edit_theta4,'string',num2str(theta4*180/pi));
set(handles.edit_d3,'string',num2str(d3));

opa=opa_pre;
the1= theta1_pre;
the2= theta1_pre;
the4= theta1_pre;
set(handles.inverse, 'Enable', 'off');
numbers = [abs(theta1-the1)/0.05, abs(theta2-the2)/0.05, abs(theta4-the4)/0.05,10];
N = max(numbers); 

for k=1:N 
     Theta_1_temp = theta1_pre + (theta1-theta1_pre)*(k)/N;
     Theta_2_temp = theta2_pre + (theta2-theta2_pre)*(k)/N;
     d_3_temp = d3_pre+ (d3-d3_pre)*(k)/N;
     Theta_4_temp = theta4_pre+ (theta4-theta4_pre)*(k)/N;
     
     drawrobot(Theta_1_temp,Theta_2_temp,d_3_temp, Theta_4_temp,opa,handles);
     pause(0.2);
end
set(handles.inverse, 'Enable', 'on');
theta1_pre = theta1;
theta2_pre = theta2;
d3_pre=d3;
theta4_pre = theta4; 

function edit_posx_Callback(hObject, eventdata, handles)

function edit_posx_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_posy_Callback(hObject, eventdata, handles)

function edit_posy_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_posz_Callback(hObject, eventdata, handles)

function edit_posz_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit9_Callback(hObject, eventdata, handles)

function edit9_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit10_Callback(hObject, eventdata, handles)

function edit10_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit11_Callback(hObject, eventdata, handles)

function edit11_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit12_Callback(hObject, eventdata, handles)

function edit12_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_theta1_Callback(hObject, eventdata, handles)

function edit_theta1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_theta2_Callback(hObject, eventdata, handles)

function edit_theta2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_d3_Callback(hObject, eventdata, handles)

function edit_d3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_theta4_Callback(hObject, eventdata, handles)

function edit_theta4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 

theta1 = (pi/180)*(get(handles.slider_theta1,'Value'));
set(handles.edit_theta1,'String',theta1*180/pi);
theta2 = (pi/180)*(get(handles.slider_theta2,'Value'));
set(handles.edit_theta2,'String',theta2*180/pi);
theta4 = (pi/180)*(get(handles.slider_theta4,'Value'));
set(handles.edit_theta4,'String',theta4*180/pi);
d3 = get(handles.slider_d3,'Value');
set(handles.edit_d3,'String',d3);
opa=0.5;

drawrobot(theta1,theta2,d3,theta4,opa, handles);
set(handles.run, 'Enable', 'off');
theta1_pre = theta1;
theta2_pre = theta2;
d3_pre=d3;
theta4_pre = theta4;



function edit18_Callback(hObject, eventdata, handles)

function edit18_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit19_Callback(hObject, eventdata, handles)

function edit19_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit20_Callback(hObject, eventdata, handles)

function edit20_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_roll_Callback(hObject, eventdata, handles)

function edit_roll_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_pitch_Callback(hObject, eventdata, handles)

function edit_pitch_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yaw_Callback(hObject, eventdata, handles)

function edit_yaw_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_c0.
function checkbox_c0_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;
% Hint: get(hObject,'Value') returns toggle state of checkbox_c0


% --- Executes on button press in checkbox_c1.
function checkbox_c1_Callback(hObject, eventdata, handles)


global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;
% Hint: get(hObject,'Value') returns toggle state of checkbox_c1


% --- Executes on button press in checkbox_c2.
function checkbox_c2_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% --- Executes on button press in checkbox_c3.
function checkbox_c3_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% --- Executes on button press in checkbox_c4.
function checkbox_c4_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% --- Executes on slider movement.
function slider_opa_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');
set(handles.edit_opa,'String',opa);

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;


function slider_opa_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit24_Callback(hObject, eventdata, handles)

function edit24_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_opa_Callback(hObject, eventdata, handles)

function edit_opa_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_workspace.
function checkbox_workspace_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;



function edit_Xm_Callback(hObject, eventdata, handles)

function edit_Xm_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_Ym_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit_Ym_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_Zm_Callback(hObject, eventdata, handles)

function edit_Zm_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in MoveL.
function MoveL_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;
opa= opa_pre;

currentX = str2double(handles.edit_posx.String);
currentY = str2double(handles.edit_posy.String);
currentZ = str2double(handles.edit_posz.String);
yaw_0= str2double(handles.edit_yaw.String);
yaw_1= str2double(handles.edit_yawm.String);

targetx = str2double(handles.edit_Xm.String);
targety = str2double(handles.edit_Ym.String);
targetz = str2double(handles.edit_Zm.String);

a1 = 450; a2= 400;
desiredVector = [targetx - currentX, targety - currentY, targetz - currentZ];
normDesiredVector = norm(desiredVector);

% qMax = normDesiredVector;  
qMax = sqrt((targetx - currentX)^2+ (targety - currentY)^2+ (targetz - currentZ)^2);
formula = @(stdVec) stdVec * desiredVector + [currentX, currentY, currentZ];

aMax = str2double(handles.edit_amax.String);
vMax= str2double(handles.edit_vmax.String);
jerk_max = str2double(handles.edit_jerkmax.String);


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

t       = 0:0.1:tmax;
lengthT = length(t);
jerk= zeros(lengthT,1);
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
Th_1=zeros(lengthT,1);
Th_2=zeros(lengthT,1);
d_3=zeros(lengthT,1);
Th_4=zeros(lengthT,1);
x_pre=currentX;
y_pre=currentY;
z_pre=currentZ;
yaw_pre=yaw_0;
X=[];
Y=[];
Z=[];
Yaw=[];

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

pause(0.2);
axes(handles.axes2);
cla(handles.axes2);
plot(t, q, 'LineWidth', 2);
xlabel('s');
ylabel('mm');
title('q(t)');
grid on;
pause(0.02);
axes(handles.axes3);
cla(handles.axes3);
plot(t, v, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s');
title('v(t)');
grid on;
pause(0.02);
axes(handles.axes4);
cla(handles.axes4);
plot(t, a, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s2');
title('a(t)');
grid on;
pause(0.02);
axes(handles.axes13);
cla(handles.axes13);
plot(t, jerk, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s3');
title('jerk(t)');
grid on;
pause(0.02);
for i = 1:1:lengthT
    qStd = q/qMax;
    desiredPos = formula(qStd(i));
    yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
    X=[X, desiredPos(1)];
    Y=[Y, desiredPos(2)];
    Z=[Z, desiredPos(3)];
    Yaw= [Yaw,yaw];

    %[Theta_1, Theta_2, d3, Theta_4] = Inverse(desiredPos(1), desiredPos(2),desiredPos(3), yaw);
    try
        [Theta_1, Theta_2, d3, Theta_4] = Inverse(desiredPos(1), desiredPos(2),desiredPos(3), yaw);
    catch exception
        msg = exception.message;
        msgbox(msg, 'Error');
        return;
    end
    Th_1(i)=Theta_1;
    Th_2(i)=Theta_2;
    Th_4(i)=Theta_4;
    d_3(i)=d3;

    v_end=[((desiredPos(1)-x_pre)/0.1);
           ((desiredPos(2)-y_pre)/0.1);
           ((desiredPos(3)-z_pre)/0.1);
           ((yaw-yaw_pre)/0.2)];
    v_x(i)= v_end(1,1);
    v_y(i)= v_end(2,1);
    v_z(i)= v_end(3,1);
    v_yaw(i)= v_end(4,1);

%     Jacobian_Matrix=[   -a2*sin(Theta_1+Theta_2)-a1*sin(Theta_1)    -a2*sin(Theta_1+Theta_2)   0   0;
%                          a2*cos(Theta_1+Theta_2)+a1*cos(Theta_1)     a2*cos(Theta_1+Theta_2)   0   0;
%                          0                                           0                         1   0;
%                          1                                           1                         0   1];
    Jacobian_Matrix=[- 450*sin(Theta_1) - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), 0, 0;
  450*cos(Theta_1) + 400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2),   400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2), 0, 0;
                                                                                 0,                                                               0, 1, 0;
                                                                                  1                                        1                       0   1];
    determinant = det(Jacobian_Matrix);
    if (determinant==0)
        msg = 'singular point';
        msgbox(msg, 'Error');
        break;
    end
    v_joint=(Jacobian_Matrix)\v_end;
    v_th1(i)=v_joint(1,1);
    v_th2(i)=v_joint(2,1);
    v_d3(i) =v_joint(3,1);
    v_th4(i)=v_joint(4,1);
    x_pre=desiredPos(1);
    y_pre=desiredPos(2);
    z_pre=desiredPos(3);
    yaw_pre=yaw;

    set(handles.slider_theta1, 'value',(Theta_1*180/pi));
    set(handles.slider_theta2, 'value',(Theta_2*180/pi));
    set(handles.slider_theta4, 'value',(Theta_4*180/pi));
    set(handles.slider_d3, 'value',d3);

    set(handles.edit_theta1,'string',num2str(Theta_1*180/pi));
    set(handles.edit_theta2,'string',num2str(Theta_2*180/pi));
    set(handles.edit_theta4,'string',num2str(Theta_4*180/pi));
    set(handles.edit_d3,'string',num2str(d3));

    
    drawrobot(Theta_1,Theta_2,d3, Theta_4,opa,handles);
    axes(handles.axes1);
    scatter3(X, Y, Z, 'r', 'filled');
    %pause(0.01);
    set(handles.edit_posx,'string',num2str(desiredPos(1)));
    set(handles.edit_posy,'string',num2str(desiredPos(2)));
    set(handles.edit_posz,'string',num2str(desiredPos(3)));
    set(handles.edit_yaw,'string',num2str(yaw));
    theta1_pre = Theta_1;
    theta2_pre = Theta_2;
    d3_pre=d3;
    theta4_pre = Theta_4; 
    
    %%%%plot joint
    axes(handles.axes5);
    cla(handles.axes5);
    plot(t(1:i), Th_1(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('Theta1(t)');
    grid on;
    axes(handles.axes6);
    cla(handles.axes6);
    plot(t(1:i), Th_2(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('theta2(t)');
    grid on;
    axes(handles.axes7);
    cla(handles.axes7);
    plot(t(1:i), d_3(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('d3(t)');
    grid on;
    axes(handles.axes8);
    cla(handles.axes8);
    plot(t(1:i), Th_4(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('Theta4(t)');
    grid on;
    axes(handles.axes9);
    cla(handles.axes9);
    plot(t(1:i), v_th1(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th1(t)');
    grid on;
    axes(handles.axes10);
    cla(handles.axes10);
    plot(t(1:i), v_th2(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th2(t)');
    grid on;
    axes(handles.axes11);
    cla(handles.axes11);
    plot(t(1:i), v_d3(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v_d3(t)');
    grid on;
    axes(handles.axes12);
    cla(handles.axes12);
    plot(t(1:i), v_th4(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th4(t)');
    grid on;

    %%%%plot tool
    axes(handles.axes14);
    cla(handles.axes14);
    plot(t(1:i), X, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('X(t)');
    grid on;
    axes(handles.axes15);
    cla(handles.axes15);
    plot(t(1:i), Y, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('Y(t)');
    grid on;
    axes(handles.axes16);
    cla(handles.axes16);
    plot(t(1:i), Z, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('Z(t)');
    grid on;
    axes(handles.axes17);
    cla(handles.axes17);
    plot(t(1:i), Yaw, 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('yaw(t)');
    grid on;
    axes(handles.axes18);
    cla(handles.axes18);
    plot(t(1:i), v_x(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('VX(t)');
    grid on;
    axes(handles.axes19);
    cla(handles.axes19);
    plot(t(1:i), v_y(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('VY(t)');
    grid on;
    axes(handles.axes20);
    cla(handles.axes20);
    plot(t(1:i), v_z(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('VZ(t)');
    grid on;
    axes(handles.axes21);
    cla(handles.axes21);
    plot(t(1:i), v_yaw(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('Vyaw(t)');
    grid on;


end


% --- Executes on button press in MoveC.
function MoveC_Callback(hObject, eventdata, handles)
% hObject    handle to MoveC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;
opa=opa_pre;
a1 = 450; a2= 400;
x_0 = str2double(handles.edit_posx.String);
y_0 = str2double(handles.edit_posy.String);
z_0 = str2double(handles.edit_posz.String);
yaw_0= str2double(handles.edit_yaw.String);
yaw_1= str2double(handles.edit_yawm.String);

x_1 = str2double(handles.edit_x1.String);
y_1 = str2double(handles.edit_y1.String);
z_1 = str2double(handles.edit_z1.String);

x_2 = str2double(handles.edit_x2.String);
y_2 = str2double(handles.edit_y2.String);
z_2 = str2double(handles.edit_z2.String);

%%%%%%%%%%%%%%
AC = [x_2-x_0;y_2-y_0;z_2-z_0];
AB = [x_1-x_0;y_1-y_0;z_1-z_0];
n = cross(AC,AB);
d = n'*[x_0;y_0;z_0];
% Tam duong tron
M = [n';...
    2*(x_1-x_0) 2*(y_1-y_0) 2*(z_1-z_0);...
    2*(x_2-x_0) 2*(y_2-y_0) 2*(z_2-z_0)];
N = [d;x_1^2+y_1^2+z_1^2-x_0^2-y_0^2-z_0^2;x_2^2+y_2^2+z_2^2-x_0^2-y_0^2-z_0^2];
O = M\N;
R= sqrt((x_1-O(1))^2+(y_1-O(2))^2 );

%%%%%%%%%%%%
alpha_0=acos((2*R*R-(x_0-O(1)-R)^2-(y_0-O(2))^2)/(2*R*R));
alpha_1=acos((2*R*R-(x_1-O(1)-R)^2-(y_1-O(2))^2)/(2*R*R));
alpha_2=acos((2*R*R-(x_2-O(1)-R)^2-(y_2-O(2))^2)/(2*R*R));
if (y_0<O(2))
    alpha_0=2*pi-alpha_0;
end
if (y_1<O(2))
    alpha_1=2*pi-alpha_1;
end
if (y_2<O(2))
    alpha_2=2*pi-alpha_2;
end
alpha= alpha_2-alpha_0;
if (alpha>0)
    if (alpha_2<pi && alpha_1>=pi)
    dau=-1;
    dentaalpha=2*pi-alpha;
    else 
       dau=1;
        dentaalpha=alpha;
      

    end

else
    if (alpha_2<pi && alpha_1<=pi)
    dau=1;
    dentaalpha=alpha;
    else
        dau=1;
    dentaalpha=2*pi+alpha;
    end
end

qMax    = R*abs(dentaalpha);

aMax = str2double(handles.edit_amax.String);
% vMax = sqrt(qMax*aMax);
vMax= str2double(handles.edit_vmax.String);
jerk_max = str2double(handles.edit_jerkmax.String);

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


t       = 0:0.2:tmax;
lengthT = length(t);
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
Th_1=zeros(lengthT,1);
Th_2=zeros(lengthT,1);
d_3=zeros(lengthT,1);
Th_4=zeros(lengthT,1);
x_pre=x_0;
y_pre=y_0;
z_pre=z_0;
yaw_pre=yaw_0;
X=[];
Y=[];
Z=[];
Yaw=[];
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

axes(handles.axes2);
cla(handles.axes2);
plot(t(1:i), q(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm');
title('q(t)');
grid on;
axes(handles.axes3);
cla(handles.axes3);
plot(t(1:i), v(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm/s');
title('v(t)');
grid on;
axes(handles.axes4);
cla(handles.axes4);
plot(t(1:i), a(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm/s2');
title('a(t)');
grid on;
axes(handles.axes13);
cla(handles.axes13);
plot(t, jerk, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s3');
title('jerk(t)');
grid on;
pause(0.02);

for i = 1:1:lengthT
    x = O(1)+R*cos(alpha_0+(q(i)/qMax)*dentaalpha*dau);
    y = O(2)+R*sin(alpha_0+(q(i)/qMax)*dentaalpha*dau);
    z = -x*n(1)/n(3)-y*n(2)/n(3)+d/n(3)-(q(i)/qMax)*(abs(z_0-z_1));
    yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
    X=[X, x];
    Y=[Y, y];
    Z=[Z, z];
    Yaw= [Yaw,yaw];
   % [Theta_1, Theta_2, d3, Theta_4] = Inverse(x, y,z, yaw);
    try
        [Theta_1, Theta_2, d3, Theta_4] = Inverse(x, y,z, yaw);
    catch exception
        msg = exception.message;
        msgbox(msg, 'Error');
        return;
    end
    Th_1(i)=Theta_1;
    Th_2(i)=Theta_2;
    Th_4(i)=Theta_4;
    d_3(i)=d3;

    v_end=[((x-x_pre)/0.2);
           ((y-y_pre)/0.2);
           ((z-z_pre)/0.2);
           ((yaw-yaw_pre)/0.2)];
    v_x(i)= v_end(1,1);
    v_y(i)= v_end(2,1);
    v_z(i)= v_end(3,1);
    v_yaw(i)= v_end(4,1);
%     Jacobian_Matrix=[   -a2*sin(Theta_1+Theta_2)-a1*sin(Theta_1)    -a2*sin(Theta_1+Theta_2)   0   0;
%                          a2*cos(Theta_1+Theta_2)+a1*cos(Theta_1)     a2*cos(Theta_1+Theta_2)   0   0;
%                          0                                           0                         1   0;
%                          1                                           1                         0   1];
    Jacobian_Matrix=[- 450*sin(Theta_1) - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), - 400*cos(Theta_1)*sin(Theta_2) - 400*cos(Theta_2)*sin(Theta_1), 0, 0;
  450*cos(Theta_1) + 400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2),   400*cos(Theta_1)*cos(Theta_2) - 400*sin(Theta_1)*sin(Theta_2), 0, 0;
                                                                                 0,                                                               0, 1, 0;
                                                                                  1                                        1                       0   1];
    determinant = det(Jacobian_Matrix);
    if (determinant==0)
        msg = 'singular point';
        msgbox(msg, 'Error');
        break;
    end
    v_joint=(Jacobian_Matrix)\v_end;
    v_th1(i)=v_joint(1,1);
    v_th2(i)=v_joint(2,1);
    v_d3(i) =v_joint(3,1);
    v_th4(i)=v_joint(4,1);
    x_pre=x;
    y_pre=y;
    z_pre=z;
    yaw_pre=yaw;

    set(handles.slider_theta1, 'value',(Theta_1*180/pi));
    set(handles.slider_theta2, 'value',(Theta_2*180/pi));
    set(handles.slider_theta4, 'value',(Theta_4*180/pi));
    set(handles.slider_d3, 'value',d3);

    set(handles.edit_theta1,'string',num2str(Theta_1*180/pi));
    set(handles.edit_theta2,'string',num2str(Theta_2*180/pi));
    set(handles.edit_theta4,'string',num2str(Theta_4*180/pi));
    set(handles.edit_d3,'string',num2str(d3));

    
    drawrobot(Theta_1,Theta_2,d3, Theta_4,opa,handles);
    axes(handles.axes1);
    scatter3(X, Y, Z, 'r', 'filled');
    pause(0.02);
    set(handles.edit_posx,'string',num2str(x));
    set(handles.edit_posy,'string',num2str(y));
    set(handles.edit_posz,'string',num2str(z));
    set(handles.edit_yaw,'string',num2str(yaw));
    theta1_pre = Theta_1;
    theta2_pre = Theta_2;
    d3_pre=d3;
    theta4_pre = Theta_4; 

    %%%%plot joint
    axes(handles.axes5);
    cla(handles.axes5);
    plot(t(1:i), Th_1(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('Theta1(t)');
    grid on;
    axes(handles.axes6);
    cla(handles.axes6);
    plot(t(1:i), Th_2(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('theta2(t)');
    grid on;
    axes(handles.axes7);
    cla(handles.axes7);
    plot(t(1:i), d_3(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('d3(t)');
    grid on;
    axes(handles.axes8);
    cla(handles.axes8);
    plot(t(1:i), Th_4(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('Theta4(t)');
    grid on;
    axes(handles.axes9);
    cla(handles.axes9);
    plot(t(1:i), v_th1(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th1(t)');
    grid on;
    axes(handles.axes10);
    cla(handles.axes10);
    plot(t(1:i), v_th2(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th2(t)');
    grid on;
    axes(handles.axes11);
    cla(handles.axes11);
    plot(t(1:i), v_d3(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v_d3(t)');
    grid on;
    axes(handles.axes12);
    cla(handles.axes12);
    plot(t(1:i), v_th4(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th4(t)');
    grid on;

    %%%plot tool
    axes(handles.axes14);
    cla(handles.axes14);
    plot(t(1:i), X, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('X(t)');
    grid on;
    axes(handles.axes15);
    cla(handles.axes15);
    plot(t(1:i), Y, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('Y(t)');
    grid on;
    axes(handles.axes16);
    cla(handles.axes16);
    plot(t(1:i), Z, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('Z(t)');
    grid on;
    axes(handles.axes17);
    cla(handles.axes17);
    plot(t(1:i), Yaw, 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('yaw(t)');
    grid on;
    axes(handles.axes18);
    cla(handles.axes18);
    plot(t(1:i), v_x(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('VX(t)');
    grid on;
    axes(handles.axes19);
    cla(handles.axes19);
    plot(t(1:i), v_y(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('VY(t)');
    grid on;
    axes(handles.axes20);
    cla(handles.axes20);
    plot(t(1:i), v_z(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('VZ(t)');
    grid on;
    axes(handles.axes21);
    cla(handles.axes21);
    plot(t(1:i), v_yaw(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('Vyaw(t)');
    grid on;

end

function edit30_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit31_Callback(hObject, eventdata, handles)

function edit31_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit32_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_x1_Callback(hObject, eventdata, handles)

function edit_x1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_y1_Callback(hObject, eventdata, handles)

function edit_y1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_z1_Callback(hObject, eventdata, handles)

function edit_z1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_x2_Callback(hObject, eventdata, handles)

function edit_x2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_y2_Callback(hObject, eventdata, handles)

function edit_y2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_z2_Callback(hObject, eventdata, handles)

function edit_z2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit39_Callback(hObject, eventdata, handles)

function edit39_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit40_Callback(hObject, eventdata, handles)

function edit40_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit41_Callback(hObject, eventdata, handles)

function edit41_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit42_Callback(hObject, eventdata, handles)

function edit42_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit43_Callback(hObject, eventdata, handles)

function edit43_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit44_Callback(hObject, eventdata, handles)

function edit44_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_vmax_Callback(hObject, eventdata, handles)

function edit_vmax_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_amax_Callback(hObject, eventdata, handles)

function edit_amax_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit47_Callback(hObject, eventdata, handles)

function edit47_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit48_Callback(hObject, eventdata, handles)

function edit48_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yawm_Callback(hObject, eventdata, handles)

function edit_yawm_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit50_Callback(hObject, eventdata, handles)

function edit50_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit51_Callback(hObject, eventdata, handles)

function edit51_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_jerkmax_Callback(hObject, eventdata, handles)

function edit_jerkmax_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_x_m_Callback(hObject, eventdata, handles)

function edit_x_m_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_y_m_Callback(hObject, eventdata, handles)

function edit_y_m_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_z_m_Callback(hObject, eventdata, handles)

function edit_z_m_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yaw_m_Callback(hObject, eventdata, handles)

function edit_yaw_m_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit57_Callback(hObject, eventdata, handles)

function edit57_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit58_Callback(hObject, eventdata, handles)

function edit58_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit59_Callback(hObject, eventdata, handles)

function edit59_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit60_Callback(hObject, eventdata, handles)

function edit60_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in move.
function move_Callback(hObject, eventdata, handles)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;
opa=opa_pre;

x1= str2double(handles.edit_posx.String);
y1= str2double(handles.edit_posy.String);
z1= str2double(handles.edit_posz.String);
yaw_1= str2double(handles.edit_yaw.String);
x2= str2double(handles.edit_x_m.String);
y2= str2double(handles.edit_y_m.String);
z2= str2double(handles.edit_z_m.String);
yaw_2=str2double(handles.edit_yaw_m.String);
a1 = 450; a2= 400;

% [the_1, the_2, d3, the_4] = Inverse(x1, y1,z1, yaw_1);
% [the_1_m, the_2_m, d3_m, the_4_m] = Inverse(x2, y2,z2, yaw_2);
try
    [the_1, the_2, d3, the_4] = Inverse(x1, y1,z1, yaw_1);
    [the_1_m, the_2_m, d3_m, the_4_m] = Inverse(x2, y2,z2, yaw_2);
catch exception
    msg = exception.message;
    msgbox(msg, 'Error');
    return;
end

qMax =sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2);
vMax = str2double(handles.edit_vmax.String);
aMax = str2double(handles.edit_amax.String);
jerk_max = str2double(handles.edit_jerkmax.String);

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

t       = 0:0.1:tmax;
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
pause(0.2);
axes(handles.axes2);
cla(handles.axes2);
plot(t, q, 'LineWidth', 2);
xlabel('s');
ylabel('mm');
title('q(t)');
grid on;
pause(0.02);
axes(handles.axes3);
cla(handles.axes3);
plot(t, v, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s');
title('v(t)');
grid on;
pause(0.02);
axes(handles.axes4);
cla(handles.axes4);
plot(t, a, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s2');
title('a(t)');
grid on;
pause(0.02);
axes(handles.axes13);
cla(handles.axes13);
plot(t, jerk, 'LineWidth', 2);
xlabel('s');
ylabel('mm/s3');
title('jerk(t)');
grid on;
pause(0.02);


theta1_pre= the_1;
theta2_pre= the_2;
theta4_pre= the_4;
d3_pre= d3;
X=[];
Y=[];
Z=[];
Yaw=[];
for i = 1:1:lengthT
    theta1= the_1+(the_1_m-the_1)*q(i)/qMax;
    theta2= the_2+(the_2_m-the_2)*q(i)/qMax;
    theta4= the_4+(the_4_m-the_4)*q(i)/qMax;
    d_3= d3+(d3_m-d3)*q(i)/qMax;
    th1(i)=theta1;
    th2(i)= theta2;
    d33(i)= d_3;
    th4(i)= theta4;

    drawrobot(theta1,theta2,d_3, theta4,opa,handles);
    %pause(0.1);
    v_joint=[((theta1-theta1_pre)/0.1);
               ((theta2-theta2_pre)/0.1);
               ((d_3-d3_pre)/0.1);
               ((theta4-theta4_pre)/0.1)];
    v_theta1(i)=v_joint(1,1);
    v_theta2(i)= v_joint(2,1);
    v_d3(i)=v_joint(3,1);
    v_theta4(i)= v_joint(4,1);

%     Jacobian_Matrix=[   -a2*sin(theta1+theta2)-a1*sin(theta1)    -a2*sin(theta1+theta2)   0   0;
%                          a2*cos(theta1+theta2)+a1*cos(theta1)     a2*cos(theta1+theta2)   0   0;
%                          0                                        0                       1   0;
%                          1                                        1                       0   1];

    Jacobian_Matrix=[- 450*sin(theta1) - 400*cos(theta1)*sin(theta2) - 400*cos(theta2)*sin(theta1), - 400*cos(theta1)*sin(theta2) - 400*cos(theta2)*sin(theta1), 0, 0;
  450*cos(theta1) + 400*cos(theta1)*cos(theta2) - 400*sin(theta1)*sin(theta2),   400*cos(theta1)*cos(theta2) - 400*sin(theta1)*sin(theta2), 0, 0;
                                                                                 0,                                                               0, 1, 0;
                                                                                  1                                        1                       0   1];
 
    v_end=(Jacobian_Matrix)*v_joint;
    v_x(i)= v_end(1,1);
    v_y(i)= v_end(2,1);
    v_z(i)= v_end(3,1);
    v_yaw(i)= v_end(4,1);

    set(handles.slider_theta1, 'value',(theta1*180/pi));
    set(handles.slider_theta2, 'value',(theta2*180/pi));
    set(handles.slider_theta4, 'value',(theta4*180/pi));
    set(handles.slider_d3, 'value',d_3);

    set(handles.edit_theta1,'string',num2str(theta1*180/pi));
    set(handles.edit_theta2,'string',num2str(theta2*180/pi));
    set(handles.edit_theta4,'string',num2str(theta4*180/pi));
    set(handles.edit_d3,'string',num2str(d_3));

    [T10 T20 T30 T40] = forward(theta1, theta2, d_3, theta4);
    X=[X, T40(1,4)];
    Y=[Y, T40(2,4)];
    Z=[Z, T40(3,4)];
    yaw = yaw_1+(q(i)/qMax)*(yaw_2-yaw_1);
    Yaw= [Yaw,yaw];
    axes(handles.axes1);
    scatter3(X, Y, Z, 'r', 'filled');
    set(handles.edit_posx,'string',num2str(T40(1,4)));
    set(handles.edit_posy,'string',num2str(T40(2,4)));
    set(handles.edit_posz,'string',num2str(T40(3,4)));
    set(handles.edit_yaw,'string',num2str(yaw));

    theta1_pre= theta1;
    theta2_pre= theta2;
    theta4_pre= theta4;
    d3_pre= d_3;

    %%%plot tool
    axes(handles.axes14);
    cla(handles.axes14);
    plot(t(1:i), X, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('x(t)');
    grid on;
    axes(handles.axes15);
    cla(handles.axes15);
    plot(t(1:i), Y, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('y(t)');
    grid on;
    axes(handles.axes16);
    cla(handles.axes16);
    plot(t(1:i), Z, 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('z(t)');
    grid on;
    axes(handles.axes17);
    cla(handles.axes17);
    plot(t(1:i), Yaw, 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('yaw(t)');
    grid on;
    axes(handles.axes18);
    cla(handles.axes18);
    plot(t(1:i), v_x(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v_x(t)');
    grid on;
    axes(handles.axes19);
    cla(handles.axes19);
    plot(t(1:i), v_y(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v_y(t)');
    grid on;
    axes(handles.axes20);
    cla(handles.axes20);
    plot(t(1:i), v_z(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v_z(t)');
    grid on;
    axes(handles.axes21);
    cla(handles.axes21);
    plot(t(1:i), v_yaw(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_yaw(t)');
    grid on;

    %%%plot joint
    axes(handles.axes5);
    cla(handles.axes5);
    plot(t(1:i), th1(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('Theta1(t)');
    grid on;
    axes(handles.axes6);
    cla(handles.axes6);
    plot(t(1:i), th2(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('theta2(t)');
    grid on;
    axes(handles.axes7);
    cla(handles.axes7);
    plot(t(1:i), d33(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('d3(t)');
    grid on;
    axes(handles.axes8);
    cla(handles.axes8);
    plot(t(1:i), th4(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad');
    title('Theta4(t)');
    grid on;
    axes(handles.axes9);
    cla(handles.axes9);
    plot(t(1:i), v_theta1(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th1(t)');
    grid on;
    axes(handles.axes10);
    cla(handles.axes10);
    plot(t(1:i), v_theta2(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th2(t)');
    grid on;
    axes(handles.axes11);
    cla(handles.axes11);
    plot(t(1:i), v_d3(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v_d3(t)');
    grid on;
    axes(handles.axes12);
    cla(handles.axes12);
    plot(t(1:i), v_theta4(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('rad/s');
    title('v_th4(t)');
    grid on;
end

%a = (gaussmf(t,[t1/8 t1/2]) - gaussmf(t,[t1/8 t2+t1/2]))*aMax;
% x= cumtrapz(t,v_x);
% y = cumtrapz(t,v_y);
% z = cumtrapz(t,v_z);
% yaww = cumtrapz(t,v_yaw);


function edit61_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit61_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in jointplan.
function jointplan_Callback(hObject, eventdata, handles)
set(handles.uipanel8, 'visible', 'on');
set(handles.uipanel9, 'visible', 'off');

% --- Executes on button press in toolplan.
function toolplan_Callback(hObject, eventdata, handles)
set(handles.uipanel8, 'visible', 'off');
set(handles.uipanel9, 'visible', 'on');
