function F = centerEstimator(sphere)

%%Sphere genearl equation (x-h)^2 + (y-j)^2 + (z-k)^2 - r^2 = 0
%%where h, j and k are the center coordintes and r is the radius of the
%%sphere

cx = sphere(1);
cy = sphere(2);
cz = sphere(3);
r = sphere(4);

global n;
global X;

for i=1:n
    F(i) = (X(1,i)-cx)^2 + (X(2,i)-cy)^2 + (X(3,i)-cz)^2 - r^2;
end



