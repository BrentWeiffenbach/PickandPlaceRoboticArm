clear;
clc;
robot = Robot();
% Transformation Matrix from checkboard to World
R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
t_0_checker = [113; -70; 0];
T_0_checker = [R_0_checker,t_0_checker;zeros(1,3), 1];
T_checker_0 = inv(T_0_checker);

% Load in our Camera Parameters.
try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
end
cameraPose = cam.getCameraPose();

% point in the camera you are trying to kind in reference to checkboard
cameraImg = imshow(cam.getImage());
POI = ginput(4);
disp("Selected Point: ");
disp(POI);

% takes image point, and puts it into checkerboard World frame
checkPoints = pointsToWorld(cam.cam_IS,cam.cam_R,cam.cam_T,POI);
disp("Point in checkboard Frame");
disp(checkPoints);

[rows,~] = size(checkPoints);


% iterate through each point and display them.
for i = 1:rows
%     WorldFramePoint = T_checker_0 * [checkPoints(i,:)'; 0; 1];
    WorldFramePoint2 = T_0_checker \ [checkPoints(i,:)'; 0; 1];
    disp("Point in World Frame: ");
%     disp(WorldFramePoint(1:3)');
    disp(WorldFramePoint2(1:3)');
end
