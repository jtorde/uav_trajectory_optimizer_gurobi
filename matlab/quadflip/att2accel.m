function a = att2accel(Fb, R, P, Y, PP)
%ATT2ACCEL Encode attitude constraint as acceleration
%   See eq 28 of Cutler, AIAA 2012

% Create desired attitude (des body w.r.t world)
Rdes = Q.fromRPY(R,P,Y).toRotm;

a = Fb/PP.mass * Rdes*[0;0;1] - [0;0;1]*PP.gravity;

end

