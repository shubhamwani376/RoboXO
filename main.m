% MAE C163A/C263A Project
% Team 15
clc;clear all;

% Initialize
initialize();
global x_tiles o_tiles tile_th i last_com_ti c;
tile_th = 1.0;
x_tiles = 3;
o_tiles = 3;
c=[20.7,14.74,13.4,11.3];
i = 1;
last_com_ti=[pi/2;pi/2;pi/2];

%Start Arduino
ardn=arduino('COM13','Mega2560');
writeDigitalPin(ardn,"D13",0)

% Main
goHome()
while 1
    disp('Player')
    disp(i)
    a=input('Choose square : 1-9');
    playerInput(a,i,ardn)
    if i==2
        i=1;
    else
        i=2;
    end
end

% Terminate
input('Press any key to terminate!');
terminate();

function goHome()
global last_com_ti port_num MX28_GOAL_POSITION PROTOCOL_VERSION MX28_ID;
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(2), MX28_GOAL_POSITION, typecast(int32((0+180)/360*4096), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(3), MX28_GOAL_POSITION, typecast(int32((-90+180)/360*4096), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(4), MX28_GOAL_POSITION, typecast(int32((90+180)/360*4096), 'uint32'));
last_com_ti=[0 90 90];
pause(5)
end

function playerInput(a,i,ardn)
global x_tiles o_tiles tile_th i last_com_ti c port_num MX28_GOAL_POSITION PROTOCOL_VERSION MX28_ID;
if i==1
    t=to_go_pos(10,x_tiles*tile_th);
    x_tiles=x_tiles-1;
else
    t=to_go_pos(12,o_tiles*tile_th);
    o_tiles=o_tiles-1;
end
    writeDigitalPin(ardn,"D13",1)
    pause(3)
    goHome()
    to_go_pos(a,tile_th)
    writeDigitalPin(ardn,"D13",0)
    pause(3)
    goHome()
end

function t=to_go_pos(a,b) 
global last_com_ti c port_num MX28_GOAL_POSITION PROTOCOL_VERSION MX28_ID;
y_offset=9.3; % robot origin to start of board (length-wise) along y
size=4; % Size of each square
z_offset=-10.4; % robot origin to start of board (width-wise) along z
x=b;
if a==1||a==2||a==3
    y=y_offset + 0.5*size;
end
if a==4||a==5||a==6
    y=y_offset + 1.5*size;
end
if a==7||a==8||a==9
    y=y_offset + 2.5*size;
end
if a==1||a==4||a==7
    z=z_offset + 0.5*size;
end
if a==2||a==5||a==8
    z=z_offset + 1.5*size;
end
if a==3||a==6||a==9
    z=z_offset + 2.5*size;
end
if a==10
    z=z_offset + 0.5*size;
    y=y_offset + 3.5*size;
end
if a==12
    z=z_offset + 2.5*size;
    y=y_offset + 3.5*size;
end
pos=[x,y+2,pi];
t=IK(pos,c);
if length(t(1,:))>1
    sum1=sum(abs(t(:,1)-last_com_ti));
    sum2=sum(abs(t(:,2)-last_com_ti));
    mi=min(sum1,sum2);
    if mi==sum1
        t=t(:,1);
    else 
        t=t(:,2);
    end
end
t(2)=-t(2);
% prismatic motor theta
r=0.8; % Radius of pulley connected to 1st servo
t0=(z)/r;
t=[t0,t'].*180/pi;
t=(t-[0,-180,-180,-180]);
disp(typecast(int32(t/360*4096), 'uint32'))
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_GOAL_POSITION, typecast(int32(round(t(1))/360*4096), 'uint32'));
pause(5)
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(2), MX28_GOAL_POSITION, typecast(int32(round(t(2))/360*4096), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(3), MX28_GOAL_POSITION, typecast(int32(round(t(3))/360*4096), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(4), MX28_GOAL_POSITION, typecast(int32(round(t(4))/360*4096), 'uint32'));
pause(5)
end