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
Tf = 10;   % [s] how long is the simulation
Ts = 0.01; % [s] simulation / control period
N = Tf/Ts; %     number of iterations
tvec = linspace(0,Tf,N);

state.pos = zeros(3,1); % [m]     intertial-frame position
state.vel = zeros(3,1); % [m/s]   body velocities
state.q = [1 0 0 0];    % (wxyz)  rotation of body w.r.t world
state.w = zeros(3,1);   % [rad/s] rate of body w.r.t world exprssd in body

% parameters
P.gravity = 9.80665; % [m/s/s]
P.mass = 3.81; % [kg]
P.J = diag([0.060224 0.122198 0.132166]);

% -------------------------------------------------------------------------
% Trajectory Generation

traj = trajgen();

% set up visualization
figure(1), clf; hold on;
hViz = viz(state, traj, [], P);

% -------------------------------------------------------------------------
% Main Simulation Loop

for i = 1:N
    t = tvec(i);
    
    hViz = viz(state, hViz, P);
    
    u = controller(state, traj, t, P);
    
    state = dynamics(state, u, Ts, P);
    drawnow;
end

