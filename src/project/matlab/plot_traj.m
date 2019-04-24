clear, clc;

% Read trajectory CSV
file = '../scripts/fliptraj.csv';
data = csvread(file);

% unpack
p = data(:,1:3);
v = data(:,4:6);
a = data(:,7:9);
j = data(:,10:12);

% timing (from optimization)
N = 40;
dt = 0.15; % period of each optim segment
dc = 0.01; % control output rate

t = 0:dc:(N*dt-dc);

figure(1), clf;
subplot(411); plot(t,p); grid on; ylabel('Position'); legend('x','y','z');
subplot(412); plot(t,v); grid on; ylabel('Velocity');
subplot(413); plot(t,a); grid on; ylabel('Acceleration');
subplot(414); plot(t,j); grid on; ylabel('Jerk');
xlabel('Time [s]');