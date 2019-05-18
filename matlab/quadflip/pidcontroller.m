function cmd = pidcontroller(state, goal, t, P)
%PIDCONTROLLER Successive-loop closure PID controller
%   See https://github.com/plusk01/nonlinearquad/blob/master/pid.ipynb

inner_desired = outer(state, goal, t, P);

u = inner(state, inner_desired, t, P);

qdes = Q.fromRPY(inner_desired(2), inner_desired(3), 0).q;
Re = Q(state.q).toRotm()' * Q(qdes).toRotm();
qe = Q.fromRotm(Re).q;

cmd.u = u;
cmd.Fi = zeros(3,1);
cmd.qdes = qdes;
cmd.wdes = [0;0;inner_desired(4)];
cmd.accel_fb = zeros(3,1);
cmd.jerk_fb = zeros(3,1);
cmd.qe = qe;

end

function y = inner(state, indes, t, P)
    persistent pid_roll;
    persistent pid_pitch;
    persistent pid_yaw_rate;
    if t == 0
        pid_roll = SimplePID(7, [], 1, -pi/6, pi/6);
        pid_pitch = SimplePID(7, [], 1, -pi/6, pi/6);
        pid_yaw_rate = SimplePID(2.5, [], [], -pi, pi);
    end
    
    % state as RPY
    RPY = Q(state.q).toRPY();
    
    % RPY error
    rollerr = indes(2) - RPY(1);
    pitcherr = indes(3) - RPY(2);
    yawdoterr = indes(4) - state.w(3);
    
    f = indes(1);
    
    [pid_roll, tph] = pid_roll.run(rollerr, P.Ts, [], []);
    [pid_pitch, tth] = pid_pitch.run(pitcherr, P.Ts, [], []);
    [pid_yaw_rate, tps] = pid_yaw_rate.run(yawdoterr, P.Ts, [], []);
    
    y = [f; tph; tth; tps];
end

function y = outer(state, goal, t, P)
    persistent pid_x;
    persistent pid_y;
    persistent pid_z;
    persistent pid_yaw;
    if t == 0
        pid_x = SimplePID(0.028, [], 0.076, -pi/12, pi/12);
        pid_y = SimplePID(0.028, [], 0.076, -pi/12, pi/12);
        pid_z = SimplePID(0.5, [], 0.5, -50, 50);
        pid_yaw = SimplePID(1, [], [], -pi/12, pi/12);
    end
    
    % position error
    e = goal.pos - state.pos;
    
    % yaw error
    RPYerr = Q(Q.Identity).toRPY() - Q(state.q).toRPY();
    
    [pid_x, th_c] = pid_x.run(e(1), P.Ts, [], []);
    [pid_y, ph_c] = pid_y.run(e(2), P.Ts, [], []);
    [pid_yaw, psdot_c] = pid_yaw.run(RPYerr(3), P.Ts, [], []);
    
    [pid_z, thrust_c] = pid_z.run(e(3), P.Ts, [], 4);
    
    z_ff = P.mass*P.gravity;
    
    y = [thrust_c + z_ff; ph_c; th_c; psdot_c];
end

