kD = 0.02;
g = 9.81;
tau_phi = 0.17;
k_phi = 1.04;
tau_theta = 0.17;
k_theta = 1.04;
A = [0 0 0 1 0 0 0 0; ...
    0 0 0 0 1 0 0 0; ...
    0 0 0 0 0 1 0 0; ...
    0 0 0 -kD 0 0 0 g; ...
    0 0 0 0 -kD 0 -g 0; ...
    0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 -1/tau_phi 0;
    0 0 0 0 0 0 0 -1/tau_theta];
B = [0 0 0; ...
    0 0 0; ...
    0 0 0; ...
    0 0 0; ...
    0 0 0; ...
    0 0 1; ...
    k_phi/tau_phi 0 0; ...
    0 k_theta/tau_theta 0];
Q = diag([50,50,80,20,20,35,20,20]);
R = diag([30,30,5]);
P = dare(A,B,Q,R);