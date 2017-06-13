data = evalin('base', 'A');
P = evalin('base', 'para0');

n = size(data);
error = 0;
for i = 1 : n(1)
    pitch = (data(i, 5) / 180) * pi;
    yaw = (data(i, 6) / 180)* pi;

    u = data(i, 1);
    v = data(i, 2);

    x_real = data(i, 3);
    y_real = data(i, 4);

    [UV] = projection(P, yaw, pitch, x_real, y_real);

    err(i) = sqrt((u - UV(1))^2 + (v - UV(2))^2);
    error = error + err(i);
end
    

fprintf('total error: %f\n', error);
