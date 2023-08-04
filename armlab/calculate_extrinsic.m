x = -0.078;
y = 2.569;
z = -9.414;
g = 9.759;

phi_z = asin(y/g);
phi_x = asin(x/g);
phi_y = asin(z/g);

Ry = [cos(phi_y) 0 sin(phi_y); 0 1 0; -sin(phi_y) 0 cos(phi_y)];
Rx = [1 0 0; 0 -cos(phi_z) -sin(phi_z); 0 sin(phi_z) -cos(phi_z)];
R = Ry*Rx;

