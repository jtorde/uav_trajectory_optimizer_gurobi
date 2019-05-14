function feasible = evalTrajFeasibility(traj, t, P)
%EVALTRAJFEASIBILITY Summary of this function goes here
%   Detailed explanation goes here

% number of trajectory segments
Nsegs = length(t)-1;

% keep track of the feasibility of each segment
feasible = zeros(1,Nsegs);

for s = 1:Nsegs

    seg.a = traj.a;
    seg.j = traj.j;
    seg.s = traj.s;
    
    [f_total M] = diffFlatGetCmds(seg, P);
    
    % NOTE: numbering and order of actuators does not really matter here
    % since we just care about the max value of each actuator. The
    % convention of this mixing matrix should be checked before being used
    % to command real motors.
    c = P.c; d = P.d;
    allocMat = [1  1  1  1;...
                d  0 -d  0;...
                0  d  0 -d;...
               -c  c -c  c];
    
    f = zeros(size(f_total,1), 4);
    for i = 1:size(f_total,1)
        f(i,:) = (allocMat \ [f_total(i);M(i,:)'])';
    end
end


end

function [f_total M] = diffFlatGetCmds(seg, P)
%DIFFFLATGETCMDS Use differential flatness property to back out forces and
%                moments from a trajectory.

% calculate collective thrust
f_total = P.mass*vecnorm(seg.a + [0 0 9.8], 2, 2);

M = zeros(size(f_total,1),3);
end