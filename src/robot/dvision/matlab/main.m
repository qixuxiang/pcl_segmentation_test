fx = 360.591090231311;
fy = 360.4918824799427;
cx = 310.7131585594641;
cy = 252.0890520277582;


% Xw2p
% Yw2p
% Zw2p

% RXw2p
% RYw2p
% RZw2p

% Xp2c
% Yp2c
% Zp2c

% RXp2c
% RYp2c
% RZp2c

% scaleYaw
% scalePitch
% biasYaw
% biasPitch

%             x y z,   rx ry rz,  x y z, rx ry rz, s s b b
Parameters = [0 0 100,  0  0  0,  0 0 0,  0  0  0, 1 1 0 0];


% TODO(MWX): How to test correctness of projection function

uv = projection(Parameters, 0, 0, 600, 0);
disp(uv);

