function [R] = rotateZ(rz)
R  = [   cos(rz)   sin(rz)         0         0;
        -sin(rz)   cos(rz)         0         0;
               0         0         1         0;
               0         0         0         1;];
end
