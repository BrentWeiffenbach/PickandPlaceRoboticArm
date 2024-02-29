clc;
clear;
clc;
robot = Robot();
model = Model(robot);
try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end
try
    pos = [422, 301]; %x, y pixel position of a selected point on the image
    disp(pos);
    % convert pixel position to world coordinates
    worldPt = pointsToWorld(cam.getCameraIntrinsics(), cam.getRotationMatrix(), cam.getTranslationVector(), pos);
    disp(worldPt);
    R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
    t_0_checker = [113; -70; 0];
    T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
    r_pos = inv(T_0_check) * [worldPt'; 0; 1];

%     r_pos = centers_to_positions(r_pos(1:2)');
    disp(r_pos);

catch exception
    getReport(exception)
    disp('Exited on error');
end
