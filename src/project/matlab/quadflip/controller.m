function cmd = controller(state, goal, t, P)
%CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

% Forces
[Fi, accel_fb] = getForce(state, goal, P);
f = norm(Fi);

% Moments
qdes = getAttitude(Fi, goal, P);
% qdes = computeAttitude(Fi);
[wdes, jerk_fb] = getRates(Fi, accel_fb, goal, qdes, t, P);

Re = Q(state.q).toRotm()' * Q(qdes).toRotm();
qe = Q.fromRotm(Re).q;

if (qe(1)<0), sgn = -1; else, sgn = 1; end
M = sgn*P.att.Kp*qe(2:4)'+ P.att.Kd*(wdes - state.w); % sign(qdes(1))*

cmd.u = [f; M];
cmd.Fi = Fi;
cmd.qdes = qdes;
cmd.wdes = wdes;
cmd.accel_fb = accel_fb;
cmd.jerk_fb = jerk_fb;
cmd.qe = qe;

end

function [Fi, accel_fb] = getForce(state, goal, P)
%GETACCEL Calculate desired force in inertial frame
    % PD control from position/velocity error to acceleration (eq 6)
    ep = goal.pos - state.pos;
    ed = goal.vel - state.vel;
    accel_fb = P.accel.Kp*ep + P.accel.Kd*ed + [0;0;1]*P.gravity;

    % equation 7: desired force in the inertial frame
    Fi = P.mass*(goal.accel + accel_fb);
    
%     F    = P.mass * (goal.accel    + goal.accel_fb);
%     F(3) = P.mass * (goal.accel(3) + P.gravity); % why no accel_fb?
end

function qdes = getAttitude(F, accel_fb, P)
%     Rdes = computeAttitude(goal.accel)';
%     qdes = quatFromRotm(Rdes);
%     return
    
    Fbar = F/norm(F); % eq 10
    Fbarb = [0;0;1]; % eq 11
    
    %
    % Find minimum-angle quaternion between Fi and Fb
    %
    
    FbxF = cross(Fbarb,Fbar);
    FbdF = dot(Fbarb,Fbar);
    qmin = [1 0 0 0];
    
    eps = 0.005;
    if norm(F) > P.minRmag
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
%     F(3) = P.mass * (goal.accel_fb(3) + P.gravity);
    
    % correct sign ambiguity if desired z accel is greater than gravity
%     if F(3)<0, F(3) = 0; end
    
    % Only assign goal attitude if you aren't close to a singularity
    if norm(F) > P.minRmag
        % TODO: make sure qmin and qmin_old agree with each other
        
        qdes = qmin;
        
        % TODO: rotate by yaw
    else
        fprintf('Singular 2\n');
        qdes = [1 0 0 0];
    end
end

function [wdes, jerk_fb] = getRates(F, accel_fb, goal, qdes, t, P)
%GETRATES Calculate desired body rates

    % Numerically differentiate acceleration feedback to get jerk_fb
    persistent last_accel_fb;
    if t ~= 0
        jerk_fb = (accel_fb - last_accel_fb) / P.Ts;
    else
        jerk_fb = zeros(size(accel_fb));
    end
    
    % LPF the differentiation
    persistent last_jerk_fb;
    if t ~= 0
        tau = 0.1;
        alpha = P.Ts / (tau + P.Ts);
        jerk_fb = (1 - alpha)*last_jerk_fb + alpha*jerk_fb;
    else
        jerk_fb = zeros(size(accel_fb));
    end

    Fbar = F/norm(F);
    Fdot = P.mass*(goal.jerk + jerk_fb);
    Fbardot = Fdot/norm(F) - F*dot(F,Fdot)/norm(F)^3;
    wdes = cross(Fbar, Fbardot);
    
%     tmp = Q(qdes).toRotm()'*Fbardot;
%     wdes = [-tmp(2); tmp(1); tmp(3)];
    
    last_accel_fb = accel_fb;
    last_jerk_fb = jerk_fb;
end

function qdes = computeAttitude(a)
% Uses the acceleration vector to reconstruct the desired attitude
% We assume a body flu coordinate frame

    % We add an acceleration in body z to counteract accel due to gravity
%     a = a + [0;0;9.81];

    % If the desired acceleration vector is zero, bail
    if sum(a) == 0
        qdes = Q.Identity;
        return;
    end

    % we only care about accel dir
    a = a/norm(a);

    % find the axis of (positive) rotation
    z = [0;0;1];
    axis = -cross(a,z);
    axis = axis/norm(axis);
    
    % angle of rotation about the axis
    angle = acos(dot(z,a));
    
    qdes = Q.fromAxisAngle(axis, angle).q;
end
