g = 9.81;
tau = 0.5;
kD = 0

A = [0 0 0 1 0 0 0 0 0;...
    0 0 0 0 1 0 0 0 0; ...
    0 0 0 0 0 1 0 0 0;...
    0 0 0 -kD 0 0 0 g 0;...
    0 0 0 0 -kD 0 -g 0 0;...
    0 0 0 0 0 -kD 0 0 0;...
    0 0 0 0 0 0 -1/tau 0 0;...
    0 0 0 0 0 0 0 -1/tau 0; ...
    0 0 0 0 0 0 0 0 0];

Bd = [0 0; 0 0; 0 0; 1 0; 0 1; 0 0; 0 0; 0 0; 0 0];

C = eye(9)

D = zeros(9,2)

sysc = ss(A, Bd, C, D);
sysd = c2d(sysc,0.05,'zoh');

sigma = zeros(9,9);
delta = zeros(9,2);
delta(1,:) = -1;
delta(2,:) = -1;
delta1 = sigma*delta
sigma_new = 0.01*eye(2);

Sigma = [sigma, delta1; delta1', sigma_new];

Abd = [sysd.A, sysd.B];

sigma = Abd * Sigma * Abd';
Sigma = [sigma, delta1; delta1', sigma_new]

for i = 1:5
    sigma = Abd * Sigma * Abd';
    delta1 = sigma*delta
    Sigma = [sigma, delta1; delta1', sigma_new]
end

x = 1.766e-08
xy = 1.102e-12
y = 3.587e-07 %8.435e-15
A = [x, xy; xy, y];


eig(A)