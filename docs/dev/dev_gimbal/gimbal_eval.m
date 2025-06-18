clc, clear

% Define rotation angles
roll  =  20 * pi/180;
pitch = -30 * pi/180;
yaw   = 120 * pi/180;

% Define symbolic axes using ZYX convention
Rx = @(x) eul2rotm([0, 0, x], "ZYX");
Ry = @(x) eul2rotm([0, x, 0], "ZYX");
Rz = @(x) eul2rotm([x, 0, 0], "ZYX");

% Full rotation matrix using ZYX sequence
R = eul2rotm([yaw, pitch, roll], "ZYX");
quat = rotm2quat(R);

% Display Euler angle extraction
disp('quat2rpy:'), disp(quat2rpy(quat) * 180/pi)
disp('rotm2eul ZYX:'), disp(fliplr(rotm2eul(R, "ZYX")) * 180/pi)
disp('quat2pry:'), disp(quat2pry(quat) * 180/pi)
disp('rotm2eul ZXY:'), disp(fliplr(rotm2eul(R, "ZXY")) * 180/pi)


%%

% Compute intermediate rotation matrices
R1 = Rz(yaw);             % yaw
R2 = R1 * Ry(pitch);      % yaw → pitch
R3 = R2 * Rx(roll);       % yaw → pitch → roll (final)

% Plotting
figure; hold on; axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
axis([-1 1 -1 1 -1 1]);
scale = 0.5;

% Base frame
quiver3(0,0,0,scale,0,0,'r','LineWidth',2); % X
quiver3(0,0,0,0,scale,0,'g','LineWidth',2); % Y
quiver3(0,0,0,0,0,scale,'b','LineWidth',2); % Z
text(0.6,0,0,'Base Frame');

% After yaw (R1)
quiver3(0,0,0,scale*R1(1,1),scale*R1(2,1),scale*R1(3,1),'r--','LineWidth',1.5); % X
quiver3(0,0,0,scale*R1(1,2),scale*R1(2,2),scale*R1(3,2),'g--','LineWidth',1.5); % Y
quiver3(0,0,0,scale*R1(1,3),scale*R1(2,3),scale*R1(3,3),'b--','LineWidth',1.5); % Z
text(0.4,0.4,0,'Yaw');

% After pitch (R2)
quiver3(0,0,0,scale*R2(1,1),scale*R2(2,1),scale*R2(3,1),'r:','LineWidth',1.5); % X
quiver3(0,0,0,scale*R2(1,2),scale*R2(2,2),scale*R2(3,2),'g:','LineWidth',1.5); % Y
quiver3(0,0,0,scale*R2(1,3),scale*R2(2,3),scale*R2(3,3),'b:','LineWidth',1.5); % Z
text(0.1,0.6,0,'Pitch');

% After roll (R3 = R)
quiver3(0,0,0,scale*R3(1,1),scale*R3(2,1),scale*R3(3,1),'r','LineWidth',4); % X
quiver3(0,0,0,scale*R3(1,2),scale*R3(2,2),scale*R3(3,2),'g','LineWidth',4); % Y
quiver3(0,0,0,scale*R3(1,3),scale*R3(2,3),scale*R3(3,3),'b','LineWidth',4); % Z
text(-0.4,0.5,0.4,'Roll');

title('3-Axis Gimbal Visualization (ZYX Convention)');
legend({'Base X','Base Y','Base Z'}, 'Location','bestoutside');
