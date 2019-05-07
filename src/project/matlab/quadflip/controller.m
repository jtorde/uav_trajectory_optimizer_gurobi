function u = controller(state, goal, t, P)
%CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
u = zeros(4,1);
end

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