function [R] = rotateY(ry)
R  = [   cos(ry)         0   sin(ry)         0;
               0         1         0         0;
        -sin(ry)         0   cos(ry)         0;
               0         0         0         1;];
end
