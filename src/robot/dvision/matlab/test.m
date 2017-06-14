fx = 360.591090231311;
fy = 360.4918824799427;
cx = 624.7131585594641;
cy = 496.0890520277582;



CameraMatrix = [fx 0 cx 0;
                0 fy cy 0;
                0  0  1 0;
                0  0  0 1];

evalin('base', 'para0');

pitch = 0;
yaw = 45 / 180 * pi;

ex = calc_extrinsic(para0, pitch, yaw);

foo = CameraMatrix * ex;


x = 240;
y = 0;

fuck = ex * [x;y;0;1]


fuck = foo * [x;y;0;1];

u = fuck(1) / fuck(3);
v = fuck(2) / fuck(3);

disp(u);
disp(v);

%fuu = inv(foo) * fuck;

u = 746;
v = 533;

bar = inv(foo);

C = bar(3,:);

s = -C(4) / (u * C(1) + v * C(2) + C(3));

x = bar(1,:) * [s*u;s*v;s;1];
y = bar(2,:) * [s*u;s*v;s;1];
