clear, clc;

% Read trajectory CSV
file = '../scripts/fliptraj.csv';
data = csvread(file);

% unpack
p = data(:,1:3);
v = data(:,4:6);
a = data(:,7:9);
j = data(:,10:12);

% timing (from optimization)
N = 40;
dt = 0.15; % period of each optim segment
dc = 0.01; % control output rate

t = 0:dc:N*dt;

figure(1), clf;
subplot(411); plot(t,p); grid on; ylabel('Position'); legend('x','y','z');
title('Multirotor Flip');
subplot(412); plot(t,v); grid on; ylabel('Velocity');
subplot(413); plot(t,a); grid on; ylabel('Acceleration');
subplot(414); plot(t,j); grid on; ylabel('Jerk');
xlabel('Time [s]');

figure(2), clf;
hold on;
for i = 1:length(t)
    
    % calculate desired attitude from acceleration vector
%     R = computeAttitude([a(i,1);a(i,2);a(i,3)]);
%     
%     plotCoordinateFrame([p(i,1);p(i,2);p(i,3)],R,0.1)
%     break;
    
    % only plot at end of segments (dt rate)
    k = mod(i,dt/dc);
%     if k ~=0 && k ~=1 && k ~=8, continue; end
    if k ~=0 && k ~= 8, continue; end
    
    % calculate desired attitude from acceleration vector
    R = computeAttitude([a(i,1);a(i,2);a(i,3)]);
    
    plotCoordinateFrame([p(i,1);p(i,2);p(i,3)],R,0.1)
    
%     if i>300, break; end
end
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Multirotor Flip')
view(0, 0); axis square; grid on;
axis([-2 1 -1 1 -1 1]*3)
pxmin = min(p(:,1))-0.25; pxmax = max(p(:,1))+0.25;
pymin = min(p(:,2))-1; pymax = max(p(:,2))+1;
pzmin = min(p(:,3))-0.25; pzmax = max(p(:,3))+0.25;
axis([pxmin pxmax pymin pymax pzmin pzmax]);

annotation('textarrow',[0.35,0.25],[0.3,0.4],'String','flip direction')

function R = computeAttitude(a)
% Uses the acceleration vector to reconstruct the desired attitude
% We assume a body flu coordinate frame

    % If the desired acceleration vector is zero, bail
    if sum(a) == 0
        R = eye(3);
        return;
    end

    % we only care about accel dir
    a = a/norm(a);

    % find the axis of (positive) rotation
    z = [0;0;1];
    axis = cross(a,z);
    axis = axis/norm(axis);
    
    % angle of rotation about the axis
    angle = acos(dot(z,a));
    
    R = axisangle2rotm(axis, angle);
end

function plotCoordinateFrame(O, R, k)
%PLOTCOORDINATEFRAME Plot a coordinate frame origin and orientation
%   Coordinate frames have a origin and an orientation. This function draws
%   the coordinate axes in a common frame.

    % k     Size of each axis

    if nargin == 1, R = eye(3); end

    % Create coordinate axes starting at 0
    kk = linspace(0,k,100);
    CX = [kk; zeros(1,length(kk)); zeros(1,length(kk))];
    CY = [zeros(1,length(kk)); kk; zeros(1,length(kk))];
    CZ = [zeros(1,length(kk)); zeros(1,length(kk)); kk];
    
    % First rotate the coordinate frame from the local frame to the
    % orientation of the desired frame in which we want to plot.
    CX = R'*CX;
    CY = R'*CY;
    CZ = R'*CZ;
    
    % Then translate this frame to its origin
    CX = repmat(O, 1, size(CX,2)) + CX;
    CY = repmat(O, 1, size(CY,2)) + CY;
    CZ = repmat(O, 1, size(CZ,2)) + CZ;
    
    % Plot the axes
    hold on; ls = '-';
    plot3(CX(1,:), CX(2,:), CX(3,:),'color','r','linewidth',2,'linestyle',ls);
    plot3(CY(1,:), CY(2,:), CY(3,:),'color','g','linewidth',2,'linestyle',ls);
    plot3(CZ(1,:), CZ(2,:), CZ(3,:),'color','b','linewidth',2,'linestyle',ls);
end

function R = axisangle2rotm(u,theta)
    % make sure axis is normalized
    u = u/norm(u);
    ux = u(1); uy = u(2); uz = u(3);
    
    r11 = cos(theta) + ux*ux*(1-cos(theta));
    r12 = ux*uy*(1-cos(theta)) - uz*sin(theta);
    r13 = ux*uz*(1-cos(theta)) + uy*sin(theta);
    r21 = uy*ux*(1-cos(theta)) + uz*sin(theta);
    r22 = cos(theta) + uy*uy*(1-cos(theta));
    r23 = uy*uz*(1-cos(theta)) - ux*sin(theta);
    r31 = uz*ux*(1-cos(theta)) - uy*sin(theta);
    r32 = uz*uy*(1-cos(theta)) + ux*sin(theta);
    r33 = cos(theta) + uz*uz*(1-cos(theta));
    
    R = [r11 r12 r13; r21 r22 r23; r31 r32 r33];
end