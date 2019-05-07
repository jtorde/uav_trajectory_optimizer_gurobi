function handle = viz(state, varargin)
%VIZ Simulation visualization and drawing tools

if nargin == 3, traj = varargin{1}; else, traj = []; end
handle = varargin{end-1};
P = varargin{end};

if isempty(handle)
    handle{1} = drawEnvironment(state, traj, P);
else
    handle{1} = drawQuad(state, handle{1});
end

end

function hQuad = drawEnvironment(state, traj, P)
    % draw quad in initial state
    hQuad = drawQuad(state, []);
    
%     for i = 1:size(traj.p,1)
%     
%         % only plot at end of segments (dt rate)
%         k = mod(i,traj.dt/P.dc);
%         if k ~= 0 && k ~= 4 && k ~= 8 && k ~= 12, continue; end
% 
%         % calculate desired attitude from acceleration vector
%         R = computeAttitude(traj.a(i,:)');
% 
%         drawCoordinateAxes(traj.p(i,:)', R, 0.1, 1);
%     end
    
    % world margin
    m = [-1 1 -1 1 -1 1]*0.75; % nominal margin
%     datalimits = [min(traj.p(:,1)) max(traj.p(:,1))...
%                   min(traj.p(:,2)) max(traj.p(:,2))...
%                   min(traj.p(:,3)) max(traj.p(:,3))];
    axis equal; grid on;
%     axis(m + datalimits)
    axis(m);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % draw coordinate axes
    view(0,0); % force axis to realize its 3d (hack!)
    A = axis;
    k = 0.2;
    O = [A(1);A(3);A(5)];
    drawCoordinateAxes(O, eye(3), k, 1);
end


function handle = drawQuad(state, handle)
    % get rotation of body w.r.t world
    R = Q(state.q).toRotm();
    
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);
    
    x2 = 0.25/2;  % half-distance along x
    y2 = 0.25/2;  % half-distance along y
    z2 = 0.025/2; % half-distance along z
    
    % vertices in body frame
    V = [...
         % top of quad rectangle
         0+x2 0+y2 0+z2;...
         0+x2 0-y2 0+z2;...
         0-x2 0-y2 0+z2;...
         0-x2 0+y2 0+z2;...
         % bottom of quad rectangle
         0+x2 0+y2 0-z2;...
         0+x2 0-y2 0-z2;...
         0-x2 0-y2 0-z2;...
         0-x2 0+y2 0-z2;...
         ];
     
     F = [...
         % top
         1 2 3 4;...
         % side facing +x
         1 5 6 2;...
         % side facing +y
         1 5 8 4;...
         % side facing -x
         3 7 8 4;...
         % side facing -y
         2 6 7 3;...
         % bottom
         5 6 7 8;...
         ];
    
    V = state.pos' + V*R';
     
    if isempty(handle)
        handle = patch('Faces',F,'Vertices',V,'FaceColor',[0.4 0.4 0.4]);
    else
        set(handle,'Faces',F,'Vertices',V);
    end
end

function handle = drawDesiredForce(state, F, handle)
    % Scale for aesthetics
    F = 1*F;
    
    % TODO: need to rotate?
    
    XX = state.pos(1)+[0 F(1)];
    YY = state.pos(2)+[0 F(2)];
    ZZ = state.pos(3)+[0 F(3)];
    
    if isempty(handle)
        handle = plot3(XX,YY,ZZ);
    else
        set(handle,'XData',XX,'YData',YY,'ZData',ZZ);
    end
end

function handle = drawDesiredAttitude(state, q, handle)
    x = state.pos(1);
    y = state.pos(2);
    z = state.pos(3);

    R = rotmFromQuat(q);
    
    handle = drawCoordinateAxes([x;y;z], R', 0.2, 1, handle);
end

function handle = drawCoordinateAxes(O, R, k, alpha, varargin)
%PLOTCOORDINATEFRAME Plot a coordinate frame origin and orientation
%   Coordinate frames have a origin and an orientation. This function draws
%   the coordinate axes in a common frame.

    % k     Size of each axis
    
    if length(varargin)==0 || isempty(varargin{1}), handle = {}; end
    if length(varargin)>=1, handle = varargin{1}; end

    % Create coordinate axes starting at 0
    kk = linspace(0,k,100);
    CX = [kk; zeros(1,length(kk)); zeros(1,length(kk))];
    CY = [zeros(1,length(kk)); kk; zeros(1,length(kk))];
    CZ = [zeros(1,length(kk)); zeros(1,length(kk)); kk];
    
    % First rotate the coordinate frame from the local frame to the
    % orientation of the desired frame in which we want to plot.
    CX = R'*CX;
    CY = R'*CY;
    CZ = R'*CZ;
    
    % Then translate this frame to its origin
    CX = repmat(O, 1, size(CX,2)) + CX;
    CY = repmat(O, 1, size(CY,2)) + CY;
    CZ = repmat(O, 1, size(CZ,2)) + CZ;
    
    % Plot the axes
    ls = '-';
    if isempty(handle)
        handle{1} = plot3(CX(1,:), CX(2,:), CX(3,:),'color','r','linewidth',2,'linestyle',ls);
        handle{2} = plot3(CY(1,:), CY(2,:), CY(3,:),'color','g','linewidth',2,'linestyle',ls);
        handle{3} = plot3(CZ(1,:), CZ(2,:), CZ(3,:),'color','b','linewidth',2,'linestyle',ls);
        
        handle{1}.Color(4) = alpha;
        handle{2}.Color(4) = alpha;
        handle{3}.Color(4) = alpha;
    else
        set(handle{1},'XData',CX(1,:),'YData',CX(2,:),'ZData',CX(3,:));
        set(handle{2},'XData',CY(1,:),'YData',CY(2,:),'ZData',CY(3,:));
        set(handle{3},'XData',CZ(1,:),'YData',CZ(2,:),'ZData',CZ(3,:));
    end
end

