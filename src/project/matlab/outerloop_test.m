%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Implementation of position goal to desired attitude from ACL snap stack.
% c.f., acl_control/src/outer_loop/control.cpp: getAccel and getAttitude.
%
% Parker Lusk
% 4 May 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;

% -------------------------------------------------------------------------
% Simulation Setup

% parameters (from acl_control SQ.yaml)
P.gravity = 9.81;
P.mass = 0.5;
P.Kp = [6.0;6.0;7.0];
P.Kd = [4.5;4.5;5.0];
P.minRmag = 0.1;

% attitude uses quaternions. flu-body and ENU-world. While 3D
% implementation, this example focuses on Y-Z plane (roll).

% current state of multirotor
state.pos = [0;0;0];
state.vel = [0;0;0];
state.q = [1 0 0 0];
state.w = [0;0;0];

% desired multirotor state
goal.pos = [5;0;3];
goal.vel = [0;0;0];
goal.accel = [0;0;0];
goal.jerk = [0;0;0];

% draw quad
figure(1), clf; hold on;
drawQuad(state, goal);

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
dc = 0.01; % control output rate

% control timesteps
tvec = 0:dc:(traj.N*traj.dt - dc);

% -------------------------------------------------------------------------
% Main Simulation Loop

for i = 1:length(tvec)
    
    t = tvec(i);
    
    % Desired multirotor state (from traj optimizer)
    goal.pos = traj.p(i,:)';
    goal.vel = traj.v(i,:)';
    goal.accel = traj.a(i,:)';
    goal.jerk = traj.j(i,:)';
    
    [F, r_nom, goal] = getAccel(state, goal, P);

    drawDesiredForce(state,F);

    qdes = getAttitude(F, r_nom, goal, P);

    drawDesiredAttitude(state,qdes);

    % I know I called this a simulation, but let's just assume that the
    % low-level controller is awesome and put us exactly where we wanted to
    % be. This also assumes that the physics of the system are on our side.
    state.pos = goal.pos;
    state.vel = goal.vel;
    state.q = qdes;
    
    if mod(i,50) == 0, drawnow; end
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
    Fbar = F/norm(F);
    Fbarb = [0;0;1];
    FbxF = cross(Fbarb,Fbar);
    FbdF = dot(Fbarb,Fbar);
    qmin = [1 0 0 0];
    
    eps = 0.05;
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

% =========================================================================
% Helper Methods
% =========================================================================

function drawQuad(state, goal)
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);
    
    x2 = 0.25/2;  % half-distance along x
    y2 = 0.25/2;  % half-distance along y
    z2 = 0.025/2; % half-distance along z
    
    V = [...
         % top of quad rectangle
         x+x2 y+y2 z+z2;...
         x+x2 y-y2 z+z2;...
         x-x2 y-y2 z+z2;...
         x-x2 y+y2 z+z2;...
         % bottom of quad rectangle
         x+x2 y+y2 z-z2;...
         x+x2 y-y2 z-z2;...
         x-x2 y-y2 z-z2;...
         x-x2 y+y2 z-z2;...
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
    
    patch('Faces',F,'Vertices',V,'FaceColor',[0.4 0.4 0.4]);
    
    % world margin
    m = [1 1 1]*1.25; % nominal margin
    m = m + abs(goal.pos);
    m = [-m(1) m(1) -m(2) m(2) -m(3) m(3)];
    axis equal; grid on;
    axis([x x y y z z]+m)
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % draw coordinate axes
    view(0,0); % force axis to realize its 3d
    A = axis;
    k = 0.2;
    O = [A(1);A(3);A(5)];
    drawCoordinateAxes(O, eye(3), k);
    
    % Look at Y-Z plane
    view(-90,0);
end

function drawDesiredForce(state, F)
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);
    
    F = 0.1*F;
    
    plot3(x+[0 F(1)],y+[0 F(2)],z+[0 F(3)]);
end

function drawDesiredAttitude(state, q)
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);

    R = rotmFromQuat(q);
    
    drawCoordinateAxes([x;y;z], R', 0.2);
end

function drawCoordinateAxes(O, R, k)
%PLOTCOORDINATEFRAME Plot a coordinate frame origin and orientation
%   Coordinate frames have a origin and an orientation. This function draws
%   the coordinate axes in a common frame.

    % k     Size of each axis

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
    plot3(CX(1,:), CX(2,:), CX(3,:),'color','r','linewidth',2,'linestyle',ls);
    plot3(CY(1,:), CY(2,:), CY(3,:),'color','g','linewidth',2,'linestyle',ls);
    plot3(CZ(1,:), CZ(2,:), CZ(3,:),'color','b','linewidth',2,'linestyle',ls);
end

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