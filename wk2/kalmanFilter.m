function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        %param.P = 0.1 * eye(4);
        param.Px = 10.*eye(2);
        param.Py = 10.*eye(2);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    % State is a four dimensional element
    %state = [x, y, vx, vy];
    dt = t - previous_t;
    Ax = [ 1 dt; ...
           0   1];
    Ay = [ 1 dt; ...
           0  1];
    sigmam = [0.01, 0; 0, 5];
    sigma0 = 0.001;
    %prediction
    updatex = Ax * [ state(1), state(3) ]';
    updatey = Ay * [ state(2), state(4) ]';
    Pxupdate = Ax*param.Px*Ax' + sigmam;
    Pyupdate = Ay*param.Px*Ay' + sigmam;
    %update
    H = [1, 0];
    xt = x - H*updatex;
    yt = y - H*updatey;
    sx = H*Pxupdate*H' + sigma0;
    sy = H*Pyupdate*H' + sigma0;
    kx = Pxupdate*H'*pinv(sx);
    ky = Pyupdate*H'*pinv(sy);
    xvalue = updatex + kx*xt;
    yvalue = updatey + ky*yt;
    param.Px = (eye(2) - kx*H)*Pxupdate;
    param.Py = (eye(2) - ky*H)*Pyupdate;
    state = [ xvalue(1), yvalue(1), xvalue(2), yvalue(2)];
    px = [1 0.330; 0 1]*xvalue;
    predictx = px(1);
    py = [1 0.330; 0 1]*yvalue;
    predicty = py(1);
end
