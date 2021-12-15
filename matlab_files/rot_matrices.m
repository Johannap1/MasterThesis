syms phi theta xi

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(xi) -sin(xi) 0; sin(xi) cos(xi) 0; 0 0 1];

syms phi_d theta_d xi_d

rot = [0; 0; xi_d] + transpose(Rz)*[0; theta_d; 0] +transpose(Rz)*transpose(Ry)*[phi_d; 0; 0];
rotM = jacobian(rot,[phi_d; theta_d; xi_d])