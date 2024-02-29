clear;
clc;
robot = Robot();
imtool close all;
close all;
% Array of color names and their respective locations in world frame
colors = ["red", robot.red; "orange", robot.orange; "yellow", robot.yellow; "green", robot.green; "gray", robot.gray; "object", robot.objectlocation];
[length, ~] = size(colors);
robot.writeGripper(true)
robot.set_joint_vars([-90,0,0,0], 2);
pause(2);
robot.getBaseImg; % gets a image of a clear checkerboard at the start of the task
prompt = "Press any Key To Start"; % wait for user to start program
input(prompt);
figure;
object_standing_still = false;
stillTime_threshold = 1;
while true
    tic;
    while ~robot.checkMovement()
        if toc > stillTime_threshold % If robot does not detect movment for 1 second
            for i = 1:length % Loop through colors and object
                color = colors(i, 1);
                centroid = robot.getCentroids(color); % Get centroid of ball or object
                if centroid(4, 1) ~= -1
                    robot.pick_and_place([centroid(1), centroid(2)], "pick") % Pick up object at centroid
                    robot.pick_and_place([str2double(colors(i, 2)), str2double(colors(i, 3))], "place"); % Place object at its pre defined location
                    break; % Breaks out of for loop
                end
                if robot.checkMovement() % If robot detects movment break so it will not immeditaly grab anything that enters the workspace
                    break; % Breaks out of for loop
                end
            end
            % Break starts here
            pause(0.1);
        end
    end
end
