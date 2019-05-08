%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
% Parker Lusk
% 6 May 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc

% -------------------------------------------------------------------------
% Simulation Setup

% timing
Tf = 4;   % [s] how long is the simulation
Ts = 0.01; % [s] simulation / control period
N = Tf/Ts; %     number of iterations
tvec = linspace(0,Tf,N);

state.pos = zeros(3,1); % [m]     intertial-frame position
state.vel = zeros(3,1); % [m/s]   inertial-frame velocities
state.q = [1 0 0 0];    % (wxyz)  rotation of body w.r.t world
state.w = zeros(3,1);   % [rad/s] rate of body w.r.t world exprssd in body

% parameters
P.drawPeriod = 0.1;
P.Ts = Ts;
P.gravity = 9.80665; % [m/s/s]
P.mass = 1.4; % [kg]
P.J = diag([0.12 0.12 0.12]);
P.bodyDrag = 0.5;
% acceleration feedback PD controller
P.accel.Kp = diag([6.0 6.0 7.0]);
P.accel.Kp = diag([1 1 1]);
P.accel.Kd = diag([4.5 4.5 5.0]);
P.accel.Kd = diag([0 0 0]);
% moments feedback PD controller
P.att.Kp = diag([0.06 0.8 0.3]);
% P.att.Kp = diag([0 0 0]);
P.att.Kd = diag([0.12 0.15 0.07]);
% P.att.Kd = diag([0.1 0.1 0.1]);
P.minRmag = 0.1;

% -------------------------------------------------------------------------
% Trajectory Generation

% path: [x xd xdd xddd xdddd; y yd ... ; z zd ...]
path.s = [0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0];
path.wps = [];
path.e = [1 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0];
[traj, ~] = trajgen(path, P);

% plot trajectory log
figure(2), clf; hold on;
plotTraj(traj, P);

% set up visualization
figure(1), clf; hold on;
hViz = viz(state, traj, [], [], P);

% -------------------------------------------------------------------------
% Main Simulation Loop

execute_path = true;

statelog = cell(N,1);
inputlog = cell(N,1);
goallog = cell(N,1);

% state.vel(1) = 0.15;
% state.q = Q.fromRPY(0,0.01,0).q;

for i = 1:N
    t = tvec(i);
    
    statelog{i} = state;
    
    goal = buildGoal(traj, i);
    cmd = controller(state, goal, t, P);
    
    hViz = viz(state, [], cmd, hViz, P);
    
    if execute_path == true
        u = cmd.u;
        state = dynamics(state, u, Ts, P);
        if mod(i,1/P.drawPeriod)==0, drawnow; end
    end
    
    inputlog{i} = cmd;
    goallog{i} = goal;
end

% Plot state log
figure(3), clf; hold on;
plotState(statelog, inputlog, goallog, tvec, P);

% =========================================================================
% Helpers
% =========================================================================

function goal = buildGoal(traj, i)
%BUILDGOAL Builds a goal struct using the ith step of a trajectory

goal.pos = traj.p(i,:)';
goal.vel = traj.v(i,:)';
goal.accel = traj.a(i,:)';
goal.jerk = traj.j(i,:)';

goal.pos = [0.5 0 0]';
goal.vel = [0 0 0]';
goal.accel = [0 0 0]';
goal.jerk = [0 0 0]';

end

function plotTraj(traj, P)
subplot(411); plot(traj.p); grid on; ylabel('Position'); legend('x','y','z');
title('Desired Trajectory');
subplot(412); plot(traj.v); grid on; ylabel('Velocity');
subplot(413); plot(traj.a); grid on; ylabel('Acceleration');
subplot(414); plot(traj.j); grid on; ylabel('Jerk');
xlabel('Time [s]');
end

function plotState(statelog, inputlog, goallog, tvec, P)

states = [statelog{:}];
p = [states.pos]';
v = [states.vel]';
q = reshape([states.q], 4, size(p,1))';
w = [states.w]';

inputs = [inputlog{:}];
u = [inputs.u]';
Fi = [inputs.Fi]';
qdes = reshape([inputs.qdes], 4, size(p,1))';
qe = reshape([inputs.qe], 4, size(p,1))';
wdes = [inputs.wdes]';
afb = [inputs.accel_fb]';
jfb = [inputs.jerk_fb]';

goals = [goallog{:}];
pdes = [goals.pos]';
vdes = [goals.vel]';

% quaternion to RPY for plotting
RPY = quat2eul(q,'ZYX') * 180/pi;
RPYdes = quat2eul(qdes,'ZYX') * 180/pi;
RPYerr = quat2eul(qe,'ZYX') * 180/pi;

% color order: make the desired and actual traces the same color
co = get(gca, 'ColorOrder');
set(groot, 'defaultAxesColorOrder', [co(1:3,:); co(1:3,:)]);

n = 7; i = 1;
subplot(n,1,i); i = i+1;
plot(tvec,p); grid on; ylabel('Position'); hold on;
plot(tvec,pdes,'--');
title('State Log'); legend('x','y','z');

subplot(n,1,i); i = i+1;
plot(tvec,v); grid on; ylabel('Velocity'); hold on;
plot(tvec,vdes,'--');

% subplot(n,1,i); i = i+1;
% plot(tvec,q(:,2:4)); grid on; ylabel('Quaternion'); hold on;
% plot(tvec,qdes(:,2:4),'--');

subplot(n,1,i); i = i+1;
plot(tvec,RPY); grid on; ylabel('Euler RPY'); hold on;
plot(tvec,RPYdes,'--');

% subplot(n,1,i); i = i+1;
% plot(tvec,RPYerr); grid on; ylabel('RPY Error');

subplot(n,1,i); i = i+1;
plot(tvec,w); grid on; ylabel('Body Rates'); hold on;
plot(tvec,wdes,'--');

subplot(n,1,i); i = i+1;
plot(tvec,u(:,1)); grid on; ylabel('Thrust');

subplot(n,1,i); i = i+1;
plot(tvec,u(:,2:4)); grid on; ylabel('Moments');

% subplot(n,1,i); i = i+1;
% plot(tvec,afb); grid on; ylabel('Accel Feedback');

subplot(n,1,i); i = i+1;
plot(tvec,jfb); grid on; ylabel('Jerk Feedback');

xlabel('Time [s]');

end