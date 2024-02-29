% Model class for plotting using Matlab

classdef Model
    properties
        robot; %stores the given robot class as robot.
        robotFigure;
        robotPlot;
        quiverHandlesX;
        quiverHandlesY;
        quiverHandlesZ;
        quiverHandlesVel;
        robotAxes;
    end
    methods
        % store robot class instance within Model as robot.
        function obj = Model(robo)
            obj.robot = robo;
        end

        % plots 1x4 array of joint angles q
        % Optional param q_dot, 1x4 joint vel array, when it exists
        % function plots velocity of end effector as well
        function plot_arm(self, q, q_dot)
            [X, Y, Z, vecPos] = self.transform_general_data(q);
       
            % plot the points
            clf;
            plot3(X,Y,Z,'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);
            grid on;
            title('Robotics Armerature 3D Representation')
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
            axis([-400 400 -400 400 -400 400]);

            
            % lambda function to reformat the stored unit vectors into
            % 1x3 matrix. Num gets each of the three directions (x,y,z) of
            % the given axis to create the axis's vector.
            pm = @(num,axis) [0,permute(vecPos(num,axis,:),[1,3,2])];

            hold on;
            % Calculate EE velocity if it exists:
            if (exist("q_dot", "var"))
                velocities = self.transform_jacobian_data(q, q_dot);
                quiver3(X(end),Y(end),Z(end),velocities(1), velocities(2), velocities(3)); % draws the velocity
                legend('Linkages','Velocity');
            else
                % draws the three joint orientations for all points.
                quiver3(X,Y,Z,pm(1,1),pm(2,1),pm(3,1)); % draws the x axis
                quiver3(X,Y,Z,pm(1,2),pm(2,2),pm(3,2)); % draws the y axis
                quiver3(X,Y,Z,pm(1,3),pm(2,3),pm(3,3)); % draws the z axis
                legend('Linkages','X','Y','Z');
            end

            % Plot the manipubalibity matrix
            manipulability = self.robot.manipulability_elipsoid(q);
            plot_ellipse(manipulability, [X(end), Y(end), Z(end)]);

            drawnow;

        end % end plot arm original
        
        % Seperate create and update functions so that the live plot isn't
        % creating a new plot every loop. Should increase general
        % efficeiency
        

        function plot = create_plot(self, q, q_dot)
            [X, Y, Z, vecPos] = self.transform_general_data(q);

            % lambda function to reformat the stored unit vectors into
            % 1x3 matrix. Num gets each of the three directions (x,y,z) of
            % the given axis to create the axis's vector.
            pm = @(num,axis) [0,permute(vecPos(num,axis,:),[1,3,2])];
            
            % plot parameters
            self.robotFigure = figure;
            self.robotAxes = axes('Parent', self.robotFigure);
            grid(self.robotAxes, 'on');
            title(self.robotAxes, 'Robotics Armerature 3D Representation');
            xlabel(self.robotAxes, 'X Axis');
            ylabel(self.robotAxes, 'Y Axis');
            zlabel(self.robotAxes, 'Z Axis');
            axis(self.robotAxes, [-400 400 -400 400 -400 400]);
            % Set an initial 3D view (adjust azimuth and elevation angles as needed)
            view(self.robotAxes, 45, 30);
            hold(self.robotAxes, 'on');

            % create and save data
            self.robotPlot = plot3(self.robotAxes, X, Y, Z, '-o', ...
                'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', [0.5,0.5,0.5]);
            self.quiverHandlesX = quiver3(self.robotAxes, X,Y,Z,pm(1,1),pm(2,1),pm(3,1)); % draws the x axis
            self.quiverHandlesY = quiver3(self.robotAxes, X,Y,Z,pm(1,2),pm(2,2),pm(3,2)); % draws the y axis
            self.quiverHandlesZ = quiver3(self.robotAxes, X,Y,Z,pm(1,3),pm(2,3),pm(3,3)); % draws the z axis
            if (exist("q_dot", "var"))
                velocities = self.transform_jacobian_data(q,q_dot);
                self.quiverHandlesVel = quiver3(X(end),Y(end),Z(end),velocities(1), velocities(2), velocities(3),"LineWidth", 3); % draws the velocity
                legend('Linkages','X', 'Y', 'Z', 'Velocity');
            else
                legend('Linkages','X','Y','Z');
            end
            % return updated initialized plot that can be updated
            plot = self;
            
        end

       function update_plot(self, q, q_dot)

            [X, Y, Z, vecPos] = self.transform_general_data(q);
            % lambda function to reformat the stored unit vectors into
            % 1x3 matrix. Num gets each of the three directions (x,y,z) of
            % the given axis to create the axis's vector.
            pm = @(num,axis) [0,permute(vecPos(num,axis,:),[1,3,2])];
    
            % set updates instead of making new quivers
            set(self.robotPlot, 'XData', X, 'YData', Y, 'ZData', Z);
            set(self.quiverHandlesX, 'Xdata', X, 'Ydata', ...
                Y, 'Zdata', Z, 'Udata', pm(1,1), 'Vdata', pm(2,1), ...
                'Wdata', pm(3,1)); % sets the x data
            set(self.quiverHandlesY, 'Xdata', X, 'Ydata', ...
                Y, 'Zdata', Z, 'Udata', pm(1,2), 'Vdata', pm(2,2), ...
                'Wdata', pm(3,2)); % Sets y data
            set(self.quiverHandlesZ, 'Xdata', X, 'Ydata', ...
                Y, 'Zdata', Z, 'Udata', pm(1,3), 'Vdata', pm(2,3), ...
                'Wdata', pm(3,3)); % Sets z data
            if (exist("q_dot", "var"))
                velocities = self.transform_jacobian_data(q, q_dot);
                set(self.quiverHandlesVel, 'Xdata', X(end), 'Ydata', ...
                Y(end), 'Zdata', Z(end), 'Udata', velocities(1), 'Vdata', ...
                velocities(2), 'Wdata', velocities(3)); % Sets velocity
            end
            drawnow;
        end

        function [X, Y, Z, vecPos] = transform_general_data(self, q)
            % create forward kinematics
            fk = self.robot.joint2fk(q);

            % creates 1x3 Matricies with X, Y, and Z points.
            X = permute(fk(1,4,:),[1,3,2]);
            Y = permute(fk(2,4,:),[1,3,2]);
            Z = permute(fk(3,4,:),[1,3,2]);

            X = [0,X];
            Y = [0,Y];
            Z = [0,Z];

            % calculate reference frames
            % Find reference frame axis
            rotMats = fk(1:3,1:3,:);
            [~, ~,sizZ] = size(rotMats);
            x = [1;0;0];
            y = [0;1;0];
            z = [0;0;1];
            
            vecPos = zeros(3,3,sizZ);
            % for each point, take it's rotation Matrix and multiply it by
            % directional unit vectors to get joint pose for each of the
            % directions
            for i = 1:sizZ
                % Multiply our direction unit vectors by the rotation
                % Matrix.
                xVec = (rotMats(:,:,i) * x);
                yVec = (rotMats(:,:,i) * y);
                zVec = (rotMats(:,:,i) * z);

                % Assign resulting values to vecPos
                vecPos(:,1,i) = xVec;
                vecPos(:,2,i) = yVec;
                vecPos(:,3,i) = zVec;
            end
        end

        function arm_point = get_arm_point(self, q)
            fk = self.robot.joint2fk(q);
            % creates 1x3 Matricies with X, Y, and Z points.
            X = permute(fk(1,4,:),[1,3,2]);
            Y = permute(fk(2,4,:),[1,3,2]);
            Z = permute(fk(3,4,:),[1,3,2]);

            X = [0,X];
            Y = [0,Y];
            Z = [0,Z];

            % plot the points for XZ Plane
            clf;
            plot(X,Z,'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);
            grid on;
            title('Robotics Armerature 3D Representation')
            xlabel('X Axis');
            ylabel('Z Axis');
            axis([-400 400 -400 400 -400 400]);

            [x_point,z_point] = ginput(1);

            % plot the points for XY Plane
            clf;
            plot(X,Y,'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);
            grid on;
            title('Robotics Armerature 3D Representation')
            xlabel('X Axis');
            ylabel('Y Axis');
            axis([-400 400 -400 400 -400 400]);

            [~, y_point] = ginput(1);
            arm_point = [x_point, y_point, z_point];

        end

        function velocities = transform_jacobian_data(self, q, q_dot)
            fdk = self.robot.vel2fdk(q, q_dot); % 6x1 p_dot matrix
            scale = 0.3;
            velocities = fdk(1:3)*scale;
%             velocities = [velocities; 0];
        end
    end % end methods

end