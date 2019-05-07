function state = dynamics(state, u, dt, P)
%DYNAMICS Nonlinear quadrotor dynamics using quaternion
%   No drag, no actuator constraints

    % break out commanded thrust and body-frame moments
    thrust = u(1);
    M = u(2:4);
    
    % TODO: saturate forces and moments

    %
    % Kinematics
    %

    % translational
    f = @(pos) Q(state.q).toRotm()*state.vel;
    state.pos = rk4(f, state.pos, dt);
  
    % rotational (the lousy way)
    f = @(q) 0.5*Q(q).mult([0 state.w']).q;
    state.q = rk4(f, state.q, dt);
    state.q = Q(state.q).normalized().q;

    %
    % Dynamics
    %

    % TODO: Add drag
    
    % force due to gravity, expressed in the inertial frame
    Fg = P.mass*[0;0;P.gravity];
    
    % body-frame thrust vector (all in body z because of motor placement)
    Fb = [0;0;thrust];
    
    % translational
    f = @(vel) (1/P.mass)*(Q(state.q).toRotm()*Fb - Fg);
    state.vel = rk4(f, state.vel, dt);
    
    % rotational
    f = @(w) inv(P.J)*(M - cross(w, P.J*w));
    state.w = rk4(f, state.w, dt);
end

