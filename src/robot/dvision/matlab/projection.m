% Created on: June 2, 2017
%    Author: Wenxing Mei <mwx36mwx@gmail.com>
function [PointImage] = projection(Parameters, yaw, pitch, x, y)
% Project a point from world onto image plane

% world to plat, R means rotate
Xw2p = Parameters(1);
Yw2p = Parameters(2);
Zw2p = Parameters(3);

RXw2p = Parameters(4);
RYw2p = Parameters(5);
RZw2p = Parameters(6); % plat to camera
Xp2c = Parameters(7);
Yp2c = Parameters(8);
Zp2c = Parameters(9);

RXp2c = Parameters(10);
RYp2c = Parameters(11);
RZp2c = Parameters(12);

scaleYaw = Parameters(13);
scalePitch = Parameters(14);
biasYaw = Parameters(15);
biasPitch = Parameters(16);

fx = evalin('base', 'fx');
fy = evalin('base', 'fy'); cx = evalin('base', 'cx');
cy = evalin('base', 'cy');

yaw = (yaw + biasYaw) * scaleYaw;
pitch = (pitch + biasPitch) * scalePitch;
PointWorld = [x; y; 0; 1];

Mw2p = rotateZ(RZw2p) ...
     * rotateY(RYw2p) ...
     * rotateX(RXw2p) ...
     * dtranslate(-Xw2p, -Yw2p, -Zw2p);

Mp2c = rotateZ(RZp2c) ...
     * rotateY(RYp2c) ...
     * rotateX(RXp2c) ...
     * dtranslate(-Xp2c, -Yp2c, -Zp2c) ...
     * rotateY(-pitch) ...
     * rotateZ(-yaw);

PC = Mp2c * Mw2p * PointWorld; % PC = PointCamera

PC = [ 0  0 -1  0;
       0  1  0  0;
       1  0  0  0;
       0  0  0  1;] * PC;


CameraMatrix = [fx 0 cx 0;
                0 fy cy 0;
                0  0  1 0];


% PointImage = CameraMatrix * [ PC(1) / PC(3);
%                               PC(2) / PC(3);
%                                           1;];

PointImage = CameraMatrix * PC;
PointImage = [PointImage(1) / PointImage(3);
              PointImage(2) / PointImage(3);];
end
