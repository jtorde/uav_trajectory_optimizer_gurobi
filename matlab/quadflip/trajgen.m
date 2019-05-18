function [traj, alphas] = trajgen(path, P)
%TRAJGEN Closed-Form Polynominal Trajectory Generation
%   Based on
%
%       Actuator Constrained Trajectory Generation and
%       Control for Variable-Pitch Quadrotors, M. Cutler, AIAA GNC 2012
%

[traj, alphas, minT] = minTimeTrajSolver(path, P);

end

function [traj, alphas, t] = minTimeTrajSolver(path, P)
%MINTIMETRAJSOLVER Follows IV.A Actuator-Constrained Minimum-Time Traj Gen

% number of waypoints
if isempty(path.wps), Nwps = 0; else, Nwps = size(path.wps,3); end

% number of segments
N = Nwps + 1;

% Polynomial coefficients
alphas = zeros(3,N*10);

% start of segment times + final time (i.e., waypoint times)
t = path.T;

% Safe segment times from the most feasible iteration
tFeasible = t;

nrIterations = 50;
stepSize = 0.1;

for it = 1:nrIterations

    fprintf('\n-- Iteration %d --\n\n', it);
    
    % Solve for the polynomial coefficients (X, Y, Z)
    for axis = 1:3
        [A, b] = buildConstraints(path, t, axis);
        fprintf('Axis %d: %e, %d\n', axis, cond(A), rank(A));
        alphas(axis,:) = A\b;
    end

    %
    % Use polynomial coefficients to generate a trajectory at desired rate
    %

    traj = getTraj(alphas, t, P);

    %
    % Check feasibility of each segment, adjusting time if necessary
    %
    
    feasible = evalTrajFeasibility(traj, t, P);
    feasible = [1 0];
    for i = 2:length(feasible)
        if feasible(i) == 1
            % save feasible times for a rainy day
            tFeasible(i) = t(i);
            
            % decrease waypoint time
            t(i) = t(i) - stepSize;
        else
            % increase waypoint time
            t(i) = t(i) + stepSize;
        end
    end
    
end
end

function [alphas, T] = polySolver(X,Y,Z,n,N,P)
    alphas = zeros(3,10);
    
    t0 = 0;
    tf = 10;
    
    t_start = t0;
    t_end = tf*2;
    t_mid = tf;
    
    iters = 1;
    
    for i = 1:iters
        eps = 0;
        ts = t0 - eps;
        te = t_mid + eps;
        if n == 1, ts = t0; end
        if n == N, te = t_mid; end
        A = get9PolySegmentMatrix(ts, te);

        b = [X; Y; Z]';
        alphas(1,:) = A\b(:,1);
        alphas(2,:) = A\b(:,2);
        alphas(3,:) = A\b(:,3);

        % check actuator feasibility
        err = calcActuatorFeasibility(alphas, 1, diag(P.J), 0.1, 0, 30, pi/6, t_mid);
%         err = 0;

        if err == 1
            t_start = t_mid;
            t_mid = (t_mid + t_end) / 2;
        else 
            t_end = t_mid;
            t_mid = (t_mid + t_start) / 2;
        end
    end
    
    % set the tf to the answer
    if err == 1
        tf = t_end;
    else 
        tf = t_mid;
    end
    
    % recalculate poly coeffs one last time
    A = get9PolySegmentMatrix(t0, tf);
    b = [X; Y; Z]';
    alphas(1,:) = A\b(:,1);
    alphas(2,:) = A\b(:,2);
    alphas(3,:) = A\b(:,3);
    
    T = tf;
end

function traj = getTraj(alphas, Tdurs, P)
%GETTRAJ Generate a trajectory from polynomial coefficients
%

% total trajectory duration
T = Tdurs(end);

% total number of points
Npts = floor(T/P.Ts);

% number of segments
Nsegs = size(alphas,2)/10;

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

% indices for trajectory
i1 = 0;
i2 = 0;

% Evaluate polynomial coeffs for each trajectory segment
for s = 1:Nsegs
    % segment duration
    t0 = Tdurs(s);
    tf = Tdurs(s+1);

    % evaluate at uniformly-spaced times throughout segment
    tvec = linspace(t0,tf,(tf-t0)/P.Ts);

    % calculate indices of this segment in overall trajectory
    % HACK(?): This is currently deleting the last point of the traj
    % segment to avoid repeated values, caused by the teps offset when
    % creating continuity constraints.
    i1 = i2 + 1;
    i2 = i1 + (length(tvec)-1) - 1;
    
    % position
    idx = (s-1)*10+(1:10);
    PX = alphas(1, idx); PY = alphas(2, idx); PZ = alphas(3, idx);
    tmp = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    pos(i1:i2,:) = tmp(2:end,:);

    % velocity
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    tmp = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    vel(i1:i2,:) = tmp(2:end,:);
    
    % acceleration
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    tmp = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    accel(i1:i2,:) = tmp(2:end,:);
    
    % jerk
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    tmp = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    jerk(i1:i2,:) = tmp(2:end,:);
    
    % snap
    PX = polyder(PX); PY = polyder(PY); PZ = polyder(PZ);
    tmp = [polyval(PX, tvec)' polyval(PY, tvec)' polyval(PZ, tvec)'];
    snap(i1:i2,:) = tmp(2:end,:);

    % keep track of end idx of this segment (for plotting)
    sidx(s) = i2;
end

% See associated HACK note, above
pos(end-Nsegs+1:end,:) = repmat(pos(end-Nsegs,:), Nsegs, 1);
vel(end-Nsegs+1:end,:) = repmat(vel(end-Nsegs,:), Nsegs, 1);
accel(end-Nsegs+1:end,:) = repmat(accel(end-Nsegs,:), Nsegs, 1);
jerk(end-Nsegs+1:end,:) = repmat(jerk(end-Nsegs,:), Nsegs, 1);
snap(end-Nsegs+1:end,:) = repmat(snap(end-Nsegs,:), Nsegs, 1);

traj.p = pos;
traj.v = vel;
traj.a = accel;
traj.j = jerk;
traj.s = snap;
traj.sidx = sidx;
traj.Tdurs = Tdurs;

end 

function [A, b] = buildConstraints(path, t, axis)
%BUILDCONSTRAINTS

% number of waypoints
if isempty(path.wps), Nwps = 0; else, Nwps = size(path.wps,3); end

% number of segments
N = Nwps + 1;

% NOTE: In the original paper, the figures indicate that the total
% trajectory times were typically < 5-10 seconds. Increasing the times (and
% hence, the values of t sent to poly9) creates numerical issues -- the
% condition number of A because very very large, and can even cause the
% matrix to numerically drop rank. It seems like there could be a better
% way to handle this...

teps = 0.001;

A = zeros(10*N,10*N);
b = zeros(10*N,1);

% starting point
firstIdx = 1:5;
for i = 1:5, A(firstIdx(i),1:10) = poly9(t(1),i-1); end
b(firstIdx) = path.s(axis,:)';

% ending point
lastIdx = size(A,1)-5+(1:5);
for i = 1:5, A(lastIdx(i),size(A,2)-10+(1:10)) = poly9(t(end),i-1); end
b(lastIdx) = path.e(axis,:)';

% start the waypoint row index
wpRowIdx = firstIdx(end);

for n = 1:Nwps
    % for convenience, extract current waypoint    
    w = path.wps(axis,:,n);

    % start the waypoint row index (starts at last row)
    wpRowIdx = wpRowIdx + (n-1)*10;

    r = 1; % row index
    d = 0; % derivative order
    while r<=10

        % If this derivative of the trajectory is specified, then the
        % second column block (coeffs of the next segment polynomial) will
        % have a positive sign so that the polynomials add to 2 times the
        % desired value. If unspecified, the continuity constraint is
        % imposed, and the sign will be negative so that their difference
        % is zero.
        if d<size(w,2) && ~any(isnan(w(d+1)))
            A(wpRowIdx+r,(n-1)*10+(1:10)) = poly9(t(n+1)-teps, d);
            A(wpRowIdx+r,(n-0)*10+(1:10)) = poly9(t(n+1)+teps, d);
            b(wpRowIdx+r) = 2*w(d+1);
            r = r+1;
        end
        
        % Continuity constraint
        A(wpRowIdx+r,(n-1)*10+(1:10)) =    poly9(t(n+1)-teps, d);
        A(wpRowIdx+r,(n-0)*10+(1:10)) = -1*poly9(t(n+1)+teps, d);
        b(wpRowIdx+r) = 0;
        
        r = r+1;
        d = d+1;
    end
end
end

function P = poly9(t, d)
%POLY9 Calculates vector of 9th-order polynomial terms without coeffs

% polynomial order
m = 9;

powers = (m-d):-1:0;
P = [arrayfun(@(x)t^x, powers) zeros(1,d)];

coeffs = ones(1,m+1);
for i = 1:d
    coeffs = polyder(coeffs);
end

% zero pad
coeffs = [coeffs zeros(1,d)];

% these coeffs are not alphas, but coeffs that come from differentiation
P = coeffs.*P;

end