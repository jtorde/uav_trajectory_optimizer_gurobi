function [Error] = calculateActuatorFeasibility(Coefficients, Mass, Moment, MaxVel, MinZForce, MaxForce, MaxAngle, tf)
%UNTITLED4 Summary of this function goes here
%   This function will test a given set of polynomials to see if they fit
%   within the actuator constraints specified
%   ESSENTIALLY
%   Coefficients: a 3X10 Matrix of coefficients
%   Mass
%   Moment - moment vector
%   tf: the final time
%MinZForce = the minumum force in the z direction typlically gravity (-45)

%the distance of each motor from the center of mass
d = 0.4;
%coefficient of drag related to yaw drag
c = 0;
%gravity
g = 9.81;
%create the moment matrix
J = [Moment(1),         0,         0;
            0,          Moment(2), 0;
            0,          0,         Moment(3)];

% Find the maximum accelerations
[maxima, foundRoots] = calculateMaximums(Coefficients, 2, tf);

%create the fTot
[~, c] = size(foundRoots);
f_total = zeros(3, c);
%now test whether these maximums are within feasible limits
for it = (1:length(foundRoots))
    %calculate the magnitude of the acceleration gravity while taking into
    %account mass and gravity
    %mag is the [0, 0, 0, F_Total]' vector of the quadrotor. It must not
    %exceed the maximum possible acceleration
    f_total(1:3, it) = Mass * (maxima(:, it) + [0; 0; g]);
    if norm(f_total(:, it)) > MaxForce || f_total(3, it) <= MinZForce
        fprintf('FORCE TOO HIGH\n');
        Error = 1;
        return
    end
end

%now use a lot of physics and math to calculate the force each motor must
%be exerting at each point

%start by declaring the variables
%inertial frame force vector
Fi = zeros(3, length(foundRoots));
Fi(1, :) = Mass * (polyval(polyder(polyder(Coefficients(1, :))), foundRoots));
Fi(2, :) = Mass * (polyval(polyder(polyder(Coefficients(2, :))), foundRoots));
% add g to the 
Fi(3, :) = Mass * (polyval(polyder(polyder(Coefficients(3, :))), foundRoots) + (g * ones(1, length(foundRoots))));

% create the first derivative of force vector
Fi_prime = zeros(3, length(foundRoots));
Fi_prime(1, :) = Mass * (polyval(polyder(polyder(polyder(Coefficients(1, :)))), foundRoots));
Fi_prime(2, :) = Mass * (polyval(polyder(polyder(polyder(Coefficients(2, :)))), foundRoots));
Fi_prime(3, :) = Mass * (polyval(polyder(polyder(polyder(Coefficients(3, :)))), foundRoots));

% create the first derivative of force vector
Fi_prime_prime = zeros(3, length(foundRoots));
Fi_prime_prime(1, :) = Mass * (polyval(polyder(polyder(polyder(polyder(Coefficients(1, :))))), foundRoots));
Fi_prime_prime(2, :) = Mass * (polyval(polyder(polyder(polyder(polyder(Coefficients(2, :))))), foundRoots));
Fi_prime_prime(3, :) = Mass * (polyval(polyder(polyder(polyder(polyder(Coefficients(3, :))))), foundRoots));

%calculate the desired omega vector
Omega_desired = zeros(3, length(foundRoots));
%Aplha desired calculation
Alpha_desired = zeros(3, length(foundRoots));
%Fi_bar_prime calc
Fi_bar_prime = zeros(3, length(foundRoots));
%Fi_bar_prime_prime calc
Fi_bar_prime_prime = zeros(3, length(foundRoots));
%create the bar vector of Fi
Fi_bar = zeros(3, length(foundRoots));
%moments
Mb = zeros(3, length(foundRoots));

%caclculate the bar and prime and alpha
for it = (1:1:length(foundRoots))
    mag = norm(Fi(:, it));
    Fi_bar(:, it) = Fi(:, it) / mag;
    
    %Calculation of fi_bar_prime
    Fi_bar_prime(:, it) = (Fi_prime(:, it) / norm(Fi(:, it))) - ((Fi(:, it) * (Fi(:, it)' * Fi_prime(:, it))) / (norm(Fi(:, it))^3));
    
    % calcing fi_bar_prime_prime
    Fi_bar_prime_prime(:, it) = (Fi_prime_prime(:, it) / norm(Fi(:, it))) - ((2 * Fi_prime(:, it) * (Fi(:, it)' * Fi_prime(:, it)) + Fi(:, it) * (Fi_prime(:, it)' * Fi_prime(:, it)) + Fi(:, it) * (Fi(:, it)' * Fi_prime_prime(:, it))) / norm(Fi(:, it))^3) + ((3 * Fi(:, it) * (Fi(:, it)' * Fi_prime(:, it))) / norm(Fi(:, it))^5);
    
    %the omega desired calc
    Omega_desired(:, it) = cross(Fi_bar(:, it), Fi_bar_prime(:, it));
    %set the z moment to zero
    Omega_desired(3, it) = 0;
    
    %the alpha vector
    Alpha_desired(:, it) = cross(Fi_bar(:, it), (Fi_bar_prime_prime(:, it) - cross(Omega_desired(:, it), (cross(Omega_desired(:, it), Fi_bar_prime(:, it))))));
    %set the z to zero
    Alpha_desired(3, it) = 0;
    
    %FINALLY! now that we have the desired omega and the simulated current
    %omega, we can calcuate the exact forces that each motor must produce at a
    %given time(the maximum accelerations along a polynomial)
    Mb(:, it) = J * Alpha_desired(:, it) + cross(Omega_desired(:, it), J * Omega_desired(:, it));
end

motorForces = zeros(4, length(Mb));

torqueMat = [ 1,  1,  1,  1; % matrix that will calculate the
                d, -d, -d,  d; % moment and motor forces necessary
                d,  d, -d, -d;
                -c,  c, -c,  c];
        
for it = (1:1:length(foundRoots))
    motorForces(:, it) = torqueMat \ [norm(f_total(:, it)); Mb(1, it); Mb(2, it); Mb(3, it)];
    
    % while in the for loop check to see if the forces are within the
    % bounds
    
    if any(motorForces(:, it) <= 1) || any(motorForces(:, it) >= MaxForce / 4)
        fprintf('INDIVIDUAL MOTOR FORCE TOO HIGH\n');
        Error = 1;
        return;
    end    
end

%IF YOU HAVE NOT RETURNED BY THIS POINT THE TRAJECTORY IS WITHIN FEASIBLE
%LIMITS
Error = -1;
return;

end

function [ maximums, foundRoots ] = calculateMaximums( startCoefficients, derivative, tf)
%UNTITLED5 Summary of this function goes here
%   this function uses a 3X10 coefficient matrix and a derivative count to
%   calculate the maximums of a given set of functions at a given
%   derivative

%calculate the Coefficient matrix for the given derivative
for it = (1:1:derivative)
   startCoefficients(1, 1:(10 - it)) = polyder(startCoefficients(1, 1:(10 - (it - 1))));
   startCoefficients(2, 1:(10 - it)) = polyder(startCoefficients(2, 1:(10 - (it - 1))));
   startCoefficients(3, 1:(10 - it)) = polyder(startCoefficients(3, 1:(10 - (it - 1))));
end

%reset the size of coeffs
Coefficients = startCoefficients(1:3, 1:(10 - derivative));

%find the roots of the next derivative and combine them into a roots vector
rootsX = roots(polyder(Coefficients(1, :)));
rootsY = roots(polyder(Coefficients(2, :)));
rootsZ = roots(polyder(Coefficients(3, :)));

%combine the roots together without repeating for optimization
foundRoots = zeros(1, 2);
index = 1;
for it = (1:1:length(rootsX))
    if isreal(rootsX(it)) && rootsX(it) <= tf && rootsX(it) >= 0
        if ~any(~(rootsX(it) - foundRoots))
            %fprintf('ADDED ROOT\n')
            foundRoots(index) = rootsX(it);
            index = index + 1;
        end
    end
end
for it = (1:1:length(rootsY))
    if isreal(rootsY(it)) && rootsY(it) <= tf && rootsY(it) >= 0
        if ~any(~(rootsY(it) - foundRoots))
            %fprintf('ADDED ROOT\n')
            foundRoots(index) = rootsY(it);
            index = index + 1;
        end
    end
end
for it = (1:1:length(rootsZ))
    if isreal(rootsZ(it)) && rootsZ(it) <= tf && rootsZ(it) >= 0
        if ~any(~(rootsZ(it) - foundRoots))
            %fprintf('ADDED ROOT\n')
            foundRoots(index) = rootsZ(it);
            index = index + 1;
        end
    end
end

maximums = zeros(3, length(foundRoots) + 1);

for it = (1:1:length(foundRoots))
    maximums(:, it) = [polyval(Coefficients(1, :), foundRoots(it));
        polyval(Coefficients(2, :), foundRoots(it));
        polyval(Coefficients(3, :), foundRoots(it))];
end

maximums(:, length(foundRoots) + 1) = [polyval(Coefficients(1, :), tf);
        polyval(Coefficients(2, :), tf);
        polyval(Coefficients(3, :), tf)];

return;

end