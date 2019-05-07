function [traj, alphas] = trajgen(path, P)
%TRAJGEN Closed-Form Polynominal Trajectory Generation
%   Based on
%
%       Actuator Constrained Trajectory Generation and
%       Control for Variable-Pitch Quadrotors, M. Cutler, AIAA GNC 2012
%

% number of waypoints
if isempty(path.wps), Nwps = 0; else, Nwps = size(path.wps,3); end

% number of segments
N = Nwps + 1;

alphas = zeros(3, 10, N);
Tsegs = zeros(1, N);

A = get9PolySegmentMatrix(0, 10);

b = [
    path.s(1,:) path.e(1,:);
    path.s(2,:) path.e(2,:);
    path.s(3,:) path.e(3,:);
    ]';

alphas(1,:,1) = A\b(:,1);
alphas(2,:,1) = A\b(:,2);
alphas(3,:,1) = A\b(:,3);
Tsegs(1) = 10;

%
% Use polynomial coefficients to generate a trajectory at the desired rate
%

traj = getTraj(alphas, Tsegs, P);

end

function traj = getTraj(alphas, Tsegs, P)
%GETTRAJ Generate a trajectory from polynomial coefficients
%

% total trajectory duration
T = sum(Tsegs);

% total number of points
Npts = T/P.Ts;

% total number of segments
Nsegs = length(Tsegs);

%
% Concatenate segments into complete trajectory
%

pos = zeros(Npts, 3);
vel = zeros(size(pos));
accel = zeros(size(pos));
jerk = zeros(size(pos));
snap = zeros(size(pos));

% trajectory index of segment ends
sidx = zeros(1, Nsegs);

% Evaluate polynomial coeffs for each trajectory segment
for s = 1:Nsegs
    % segment duration
    tf = Tsegs(s);

    % evaluate at uniformly-spaced times throughout segment
    tvec = linspace(0,tf,tf/P.Ts);

    % calculate indices of this segment in overall trajectory
    i1 = (s-1)*length(tvec) + 1;
    i2 = s*length(tvec);
    
    % position
    PX = alphas(1, 1:10, s); PY = alphas(2, 1:10, s); PZ = alphas(3, 1:10, s);
    pos(i1:i2,:) = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];

    % velocity
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    vel(i1:i2,:) = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    
    % acceleration
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    accel(i1:i2,:) = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    
    % jerk
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    jerk(i1:i2,:) = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    
    % snap
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    snap(i1:i2,:) = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];

    % keep track of end idx of this segment (for plotting)
    sidx(s) = i2;
end

traj.p = pos;
traj.v = vel;
traj.a = accel;
traj.j = jerk;
traj.s = snap;
traj.sidx = sidx;
traj.Tsegs = Tsegs;

end 

function A = get9PolySegmentMatrix(t0, tf)
%POLY9 Creates 9th-Order Polynomial matrix for a trajectory segment
%   This is used to solve the linear system of equations Ax=b, where
%   A is the 10x10 matrix of terms from a 9th order polynomial and its
%   derivatives; x is polynominal coeffs (the alphas) we are solving for;
%   b is the position of the start and end points along with their first
%   for derivatives.

A = [
    % corresponding to beginning of segment (at time t0)
    t0^9,        t0^8,         t0^7,         t0^6,         t0^5,       t0^4,     t0^3,   t0^2, t0, 1;
    9*t0^8,       8*t0^7,       7*t0^6,       6*t0^5,       5*t0^4,     4*t0^3,   3*t0^2, 2*t0, 1,  0;
    8*9*t0^7,     7*8*t0^6,     6*7*t0^5,     5*6*t0^4,     4*5*t0^3,   3*4*t0^2, 2*3*t0, 1,    0,  0;
    7*8*9*t0^6,   6*7*8*t0^5,   5*6*7*t0^4,   4*5*6*t0^3,   3*4*5*t0^2, 2*3*4*t0, 1,      0,    0,  0;
    6*7*8*9*t0^5, 5*6*7*8*t0^4, 4*5*6*7*t0^3, 3*4*5*6*t0^2, 2*3*4*5*t0, 1,        0,      0,    0,  0;
    % corresponding to end of segment (at time tf)
    tf^9,         tf^8,         tf^7,         tf^6,         tf^5,       tf^4,     tf^3,   tf^2, tf, 1;
    9*tf^8,       8*tf^7,       7*tf^6,       6*tf^5,       5*tf^4,     4*tf^3,   3*tf^2, 2*tf, 1,  0;
    8*9*tf^7,     7*8*tf^6,     6*7*tf^5,     5*6*tf^4,     4*5*tf^3,   3*4*tf^2, 2*3*tf, 1,    0,  0;
    7*8*9*tf^6,   6*7*8*tf^5,   5*6*7*tf^4,   4*5*6*tf^3,   3*4*5*tf^2, 2*3*4*tf, 1,      0,    0,  0;
    6*7*8*9*tf^5, 5*6*7*8*tf^4, 4*5*6*7*tf^3, 3*4*5*6*tf^2, 2*3*4*5*tf, 1,        0,      0,    0,  0;
    ];
end