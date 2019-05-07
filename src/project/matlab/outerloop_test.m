%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Implementation of position goal to desired attitude from ACL snap stack.
% c.f., acl_control/src/outer_loop/control.cpp: getAccel and getAttitude.
%
%
% attitude uses quaternions. flu-body and ENU-world.
%
%
% Parker Lusk
% 4 May 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;

% -------------------------------------------------------------------------
% Load trajectory

% Read trajectory CSV
file = '../scripts/fliptraj.csv';
data = csvread(file);

% unpack trajectory (goal)
traj.p = data(:,1:3);
traj.v = data(:,4:6);
traj.a = data(:,7:9);
traj.j = data(:,10:12);

% timing (from optimization)
traj.N = 40;
traj.dt = 0.15; % period of each optim segment

% -------------------------------------------------------------------------
% Simulation Setup

% parameters (from acl_control SQ.yaml)
P.gravity = 9.81;
P.mass = 0.5;
P.Kp = [6.0;6.0;7.0];
P.Kd = [4.5;4.5;5.0];
P.minRmag = 0.1;
P.dc = 0.01; % control output rate

% control timesteps
tvec = 0:P.dc:(traj.N*traj.dt - P.dc);

% current state of multirotor
state.pos = traj.p(1,:)'; %[0;0;0];
state.vel = traj.v(1,:)'; %[0;0;0];
state.q = [1 0 0 0]; % this isn't really true...

% Plot signal profiles of trajectory
figure(2), clf;
subplot(411); plot(tvec,traj.p); grid on; ylabel('Position'); legend('x','y','z');
title('Multirotor Flip');
subplot(412); plot(tvec,traj.v); grid on; ylabel('Velocity');
subplot(413); plot(tvec,traj.a); grid on; ylabel('Acceleration');
subplot(414); plot(tvec,traj.j); grid on; ylabel('Jerk');
xlabel('Time [s]');

% draw environment
figure(1), clf; hold on;
hQuad = drawEnvironment(state, traj, P);

hForce = [];
hDesAtt = {};

% -------------------------------------------------------------------------
% Main Simulation Loop

for i = 1:length(tvec)
    t = tvec(i);
    
    % Update quadrotor drawing
    hQuad = drawQuad(state, hQuad);
    
    % Desired multirotor state (from traj optimizer)
    goal.pos = traj.p(i,:)';
    goal.vel = traj.v(i,:)';
    goal.accel = traj.a(i,:)';% + [0;0;9.81];
    goal.jerk = traj.j(i,:)';
    
    [F, r_nom, goal] = getAccel(state, goal, P);

    hForce = drawDesiredForce(state,F,hForce);

    qdes = getAttitude(F, r_nom, goal, P);

    hDesAtt = drawDesiredAttitude(state,qdes,hDesAtt);

    % I know I called this a simulation, but let's just assume that the
    % low-level controller is awesome and put us exactly where we wanted to
    % be. This also assumes that the physics of the system are on our side.
    state.pos = goal.pos;
    state.vel = goal.vel;
    state.q = qdes;
    
    if mod(i,5) == 0, drawnow; end
end

% =========================================================================
% Controlllers
% =========================================================================

function [F, r_nom, goal] = getAccel(state, goal, P)
    % PD control from position/velocity error to acceleration
    e = goal.pos - state.pos;
    edot = goal.vel - state.vel;
    goal.accel_fb = P.Kp.*e + P.Kd.*edot;
    
    r_nom = P.mass * (goal.accel + [0;0;1]*P.gravity);
    
    % TODO: numerically differentiate feedback acceleration to get jerk_fb
    
    F    = P.mass * (goal.accel    + goal.accel_fb);
    F(3) = P.mass * (goal.accel(3) + P.gravity); % why no accel_fb?
end

function qdes = getAttitude(F, r_nom, goal, P)
%     Rdes = computeAttitude(goal.accel)';
%     qdes = quatFromRotm(Rdes);
%     return
    
    Fbar = F/norm(F);
    Fbarb = [0;0;1];
    FbxF = cross(Fbarb,Fbar);
    FbdF = dot(Fbarb,Fbar);
    qmin = [1 0 0 0];
    
    eps = 0.005;
    if norm(r_nom) > P.minRmag
        if FbdF + 1 > eps
            tmp = 1/sqrt(2*(1 + FbdF));
            qmin(2:4) = tmp*FbxF';
        else
            fprintf('Singluar 0\n');
        end
    else
        fprintf('Singular 1\n');
    end
    qmin = qmin/norm(qmin);
    
    % recompute r3 using true desired acceleration -- why?
    F(3) = P.mass * (goal.accel_fb(3) + P.gravity);
    
    % correct sign ambiguity if desired z accel is greater than gravity
    if F(3)<0, F(3) = 0; end
    
    goal.f_total = norm(F);
    
    % Only assign goal attitude if you aren't close to a singularity
    if norm(r_nom) > P.minRmag
        % TODO: make sure qmin and qmin_old agree with each other
        
        qdes = qmin;
        
        % TODO: rotate by yaw
    else
        fprintf('Singular 2\n');
        qdes = [1 0 0 0];
    end
end

function R = computeAttitude(a)
% Uses the acceleration vector to reconstruct the desired attitude
% We assume a body flu coordinate frame

    % We add an acceleration in body z to counteract accel due to gravity
    a = a + [0;0;9.81];

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

% =========================================================================
% Drawing Helpers
% =========================================================================

function hQuad = drawEnvironment(state, traj, P)
    % draw quad in initial state
    hQuad = drawQuad(state, []);
    
    for i = 1:size(traj.p,1)
    
        % only plot at end of segments (dt rate)
        k = mod(i,traj.dt/P.dc);
        if k ~= 0 && k ~= 4 && k ~= 8 && k ~= 12, continue; end

        % calculate desired attitude from acceleration vector
        R = computeAttitude(traj.a(i,:)');

        drawCoordinateAxes(traj.p(i,:)', R, 0.1, 1);
    end
    
    % world margin
    m = [-1 1 -1 1 -1 1]*0.75; % nominal margin
    datalimits = [min(traj.p(:,1)) max(traj.p(:,1))...
                  min(traj.p(:,2)) max(traj.p(:,2))...
                  min(traj.p(:,3)) max(traj.p(:,3))];
    axis equal; grid on;
    axis(m + datalimits)
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % draw coordinate axes
    view(0,0); % force axis to realize its 3d (hack!)
    A = axis;
    k = 0.2;
    O = [A(1);A(3);A(5)];
    drawCoordinateAxes(O, eye(3), k, 1);
end


function handle = drawQuad(state, handle)
    % get rotation of body w.r.t world
    R = rotmFromQuat(state.q);
    
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);
    
    x2 = 0.25/2;  % half-distance along x
    y2 = 0.25/2;  % half-distance along y
    z2 = 0.025/2; % half-distance along z
    
    % vertices in body frame
    V = [...
         % top of quad rectangle
         0+x2 0+y2 0+z2;...
         0+x2 0-y2 0+z2;...
         0-x2 0-y2 0+z2;...
         0-x2 0+y2 0+z2;...
         % bottom of quad rectangle
         0+x2 0+y2 0-z2;...
         0+x2 0-y2 0-z2;...
         0-x2 0-y2 0-z2;...
         0-x2 0+y2 0-z2;...
         ];
     
     F = [...
         % top
         1 2 3 4;...
         % side facing +x
         1 5 6 2;...
         % side facing +y
         1 5 8 4;...
         % side facing -x
         3 7 8 4;...
         % side facing -y
         2 6 7 3;...
         % bottom
         5 6 7 8;...
         ];
    
    V = state.pos' + V*R';
     
    if isempty(handle)
        handle = patch('Faces',F,'Vertices',V,'FaceColor',[0.4 0.4 0.4]);
    else
        set(handle,'Faces',F,'Vertices',V);
    end
end

function handle = drawDesiredForce(state, F, handle)
    % Scale for aesthetics
    F = 1*F;
    
    % TODO: need to rotate?
    
    XX = state.pos(1)+[0 F(1)];
    YY = state.pos(2)+[0 F(2)];
    ZZ = state.pos(3)+[0 F(3)];
    
    if isempty(handle)
        handle = plot3(XX,YY,ZZ);
    else
        set(handle,'XData',XX,'YData',YY,'ZData',ZZ);
    end
end

function handle = drawDesiredAttitude(state, q, handle)
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);

    R = rotmFromQuat(q);
    
    handle = drawCoordinateAxes([x;y;z], R', 0.2, 1, handle);
end

function handle = drawCoordinateAxes(O, R, k, alpha, varargin)
%PLOTCOORDINATEFRAME Plot a coordinate frame origin and orientation
%   Coordinate frames have a origin and an orientation. This function draws
%   the coordinate axes in a common frame.

    % k     Size of each axis
    
    if length(varargin)==0 || isempty(varargin{1}), handle = {}; end
    if length(varargin)>=1, handle = varargin{1}; end

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
    ls = '-';
    if isempty(handle)
        handle{1} = plot3(CX(1,:), CX(2,:), CX(3,:),'color','r','linewidth',2,'linestyle',ls);
        handle{2} = plot3(CY(1,:), CY(2,:), CY(3,:),'color','g','linewidth',2,'linestyle',ls);
        handle{3} = plot3(CZ(1,:), CZ(2,:), CZ(3,:),'color','b','linewidth',2,'linestyle',ls);
        
        handle{1}.Color(4) = alpha;
        handle{2}.Color(4) = alpha;
        handle{3}.Color(4) = alpha;
    else
        set(handle{1},'XData',CX(1,:),'YData',CX(2,:),'ZData',CX(3,:));
        set(handle{2},'XData',CY(1,:),'YData',CY(2,:),'ZData',CY(3,:));
        set(handle{3},'XData',CZ(1,:),'YData',CZ(2,:),'ZData',CZ(3,:));
    end
end

% =========================================================================
% Rotation Representations
% =========================================================================

function R = rotmFromQuat( q )
% qGetR: get a 3x3 rotation matrix
% R = qGetR( Qrotation )
% IN: 
%     Qrotation - quaternion describing rotation
% 
% OUT:
%     R - rotation matrix 
%     
% VERSION: 03.03.2012

% https://www.mathworks.com/matlabcentral/fileexchange/35475-quaternions

w = q( 1 );
x = q( 2 );
y = q( 3 );
z = q( 4 );

Rxx = 1 - 2*(y^2 + z^2);
Rxy = 2*(x*y - z*w);
Rxz = 2*(x*z + y*w);

Ryx = 2*(x*y + z*w);
Ryy = 1 - 2*(x^2 + z^2);
Ryz = 2*(y*z - x*w );

Rzx = 2*(x*z - y*w );
Rzy = 2*(y*z + x*w );
Rzz = 1 - 2 *(x^2 + y^2);

R = [ 
    Rxx,    Rxy,    Rxz;
    Ryx,    Ryy,    Ryz;
    Rzx,    Rzy,    Rzz];
end

function Q = quatFromRotm( R )
% qGetQ: converts 3x3 rotation matrix into equivalent quaternion
% Q = qGetQ( R );

% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

[r,c] = size( R );
if( r ~= 3 | c ~= 3 )
    fprintf( 'R must be a 3x3 matrix\n\r' );
    return;
end

% [ Rxx, Rxy, Rxz ] = R(1,1:3); 
% [ Ryx, Ryy, Ryz ] = R(2,1:3);
% [ Rzx, Rzy, Rzz ] = R(3,1:3);

Rxx = R(1,1); Rxy = R(1,2); Rxz = R(1,3);
Ryx = R(2,1); Ryy = R(2,2); Ryz = R(2,3);
Rzx = R(3,1); Rzy = R(3,2); Rzz = R(3,3);

w = sqrt( trace( R ) + 1 ) / 2;

% check if w is real. Otherwise, zero it.
if( imag( w ) > 0 )
     w = 0;
end

x = sqrt( 1 + Rxx - Ryy - Rzz ) / 2;
y = sqrt( 1 + Ryy - Rxx - Rzz ) / 2;
z = sqrt( 1 + Rzz - Ryy - Rxx ) / 2;

[element, i ] = max( [w,x,y,z] );

if( i == 1 )
    x = ( Rzy - Ryz ) / (4*w);
    y = ( Rxz - Rzx ) / (4*w);
    z = ( Ryx - Rxy ) / (4*w);
end

if( i == 2 )
    w = ( Rzy - Ryz ) / (4*x);
    y = ( Rxy + Ryx ) / (4*x);
    z = ( Rzx + Rxz ) / (4*x);
end

if( i == 3 )
    w = ( Rxz - Rzx ) / (4*y);
    x = ( Rxy + Ryx ) / (4*y);
    z = ( Ryz + Rzy ) / (4*y);
end

if( i == 4 )
    w = ( Ryx - Rxy ) / (4*z);
    x = ( Rzx + Rxz ) / (4*z);
    y = ( Ryz + Rzy ) / (4*z);
end

Q = [ w x y z ];
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