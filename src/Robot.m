
classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        L2Angle; % angle between two and three (deg)
        T_0_1; % transformation from world to joint 1
        joint_1_2_SymsMatrix;
        joint_1_3_SymsMatrix;
        joint_1_4_SymsMatrix;
        joint_1_ee_SymsMatrix;
        jacobian_sym;
        velOffset;

        % Color Locations
        red;
        orange;
        yellow;
        green;
        gray;
        objectlocation;

        % Camera Variables
        T_0_checker;
        T_checker_0;
        cam;
        camheight;
        cam2check;
        base2check;
        ballradius;


    end
    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            % Robot Dimensions
            self.mDim = [36.076; % World to joint 1
                60.25; % joint 1 to joint 2
                130.23; % Joint 2 to joint 3
                124; % joint 3 to joint 4
                133.4 ]'; % joint 4 to end effector
            % all in (mm)

            self.mOtherDim = [128, 24]; % (mm)

            % constants or image processing in mm
            self.camheight = 188;
            self.cam2check = 275;
            self.base2check = 113;
            self.ballradius = 12.5;

            self.L2Angle = 79.3792; % (deg)

            self.velOffset = [61.83 65.952];

            self.T_0_1 = [  1 0 0 0;
                            0 1 0 0;
                            0 0 1 self.mDim(1);
                            0 0 0 1 ];

            % Locations to put found objects in
            self.red = [50 -195];
            self.orange = [130 -195];
            self.yellow = [50 195];
            self.green = [130 195];
            self.gray = [-50 -195];
            self.objectlocation = [-100, -100];

            % Define intermediate transformation matrices for jacobian calcs
            self.jacobian_sym = simplify(self.get_jacobian_syms());

            % rotation, and translation Matricies of the checkerboard
            % and world frame.
            R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
            t_0_checker = [110; -90; 0];

            % transformation from world to checker and vise versa
            self.T_0_checker = [R_0_checker,t_0_checker;zeros(1,3), 1];
            self.T_checker_0 = inv(self.T_0_checker);

            % load camera using calibrated parameters on intialization
            try
                load("camParams.mat");
                disp("Loaded Camera Parameters from camParams.mat");
                self.cam = cam;
            catch exception
                disp("Could not find camParams.mat, creating new Camera object");
            end
            

        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.

        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;
            % Uncomment to show time and acc time
            % disp("time")
            % disp(time_ms)
            % disp("acc time")
            % disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end

        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'}
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);

            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        % Takes a 1x4 array of joint variables (in degrees) to be sent to the actuators
        % accept an additional optional parameter that specifies the
        % desired travel time in miliseconds
        function set_joint_vars(self, jointVars, travelTime)
            if(exist("travelTime"))
                self.writeTime(travelTime);
            else
                self.writeTime(0);
            end
            self.writeJoints(jointVars);
        end

        % uses booleans GETPOS and GETVEL to request data
        % returns the results for the requested data, and set the rest to zero.
        % returns 2 x 4 matrix, each row corresponding to positions and
        % velocities
        function joint_vars = read_joint_vars(self, GETPOS, GETVEL)
            joint_vars = zeros(2, 4);
            if(GETPOS == true)
                joint_vars(1,:) =  (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end
            if(GETVEL == true)
                joint_vars(2,:) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end
        end

        % Forward Kinematic Functions

        % dh2mat Takes a DH table row and outputs a transformation matrix to the intermediate frame
        function t_matrix = dh2mat(self, DH_row)
            % Define DH parameters
            theta = DH_row(1);
            d = DH_row(2);
            a = DH_row(3);
            alpha = DH_row(4);

            % Make DH matrix
            t_matrix = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
                sind(theta) cosd(theta)*cosd(alpha)  -cosd(theta)*sind(alpha) a*sind(theta);
                0,sind(alpha),cosd(alpha),d;
                0,0,0,1];

            % TESTING CHECK: check if transformation matrix is valid
            %             determinent = round(det(t_matrix),4);
            %             if determinent ~= 1
            %                 t_matrix = "not a valid transformation matrix! MAJOR ERROR!";
            %             end
        end

        % takes in a 1x4 joint_vars array of joint angles (in degrees) and outputs the forward kinematics transformation matrix for the robot
        function fk = joint2fk(self, joint_vars, symbol)
            if (exist("symbol"))
                symbol = true;
            else
                symbol = false;
            end
            link1 = [joint_vars(1)             self.mDim(2) 0 -90]; % First row of DH Table
            link2 = [(-self.L2Angle + joint_vars(2)) 0 self.mDim(3) 0]; % Second row of DH Table
            link3 = [(self.L2Angle + joint_vars(3))  0 self.mDim(4) 0]; % Third row of DH Table
            link4 = [joint_vars(4)             0 self.mDim(5) 0]; % Fourth row of DH Table

            dhTable = [link1; link2; link3; link4]; % Create final DH Table

            if symbol
                fk = self.dh2fksym(dhTable);
            else 
                fk = self.dh2fk(dhTable);
            end
            % fk = transMatrices; % takes final transformation matrix

        end

        % takes a full DH_Table and outputs the transformation matrix from the base to the tip of the end effector
        function t_trans_mat = dh2fk(self, DH_Table)
            joint_1_2_Matrix = self.dh2mat(DH_Table(1,:)); % matrix transformation from joint 1 to 2
            joint_2_3_Matrix = self.dh2mat(DH_Table(2,:)); % matrix transformation from joint 2 to 3
            joint_3_4_Matrix = self.dh2mat(DH_Table(3,:)); % matrix transformation from joint 3 to 4
            joint_4_ee_Matrix = self.dh2mat(DH_Table(4,:)); % matrix transformation from joint 4 to end effector

            t_trans_mat(:,:,1) = self.T_0_1;
            t_trans_mat(:,:,2) = self.T_0_1 * joint_1_2_Matrix;
            t_trans_mat(:,:,3) = t_trans_mat(:,:,2) * joint_2_3_Matrix;
            t_trans_mat(:,:,4) = t_trans_mat(:,:,3) * joint_3_4_Matrix;
            t_trans_mat(:,:,5) = t_trans_mat(:,:,4) * joint_4_ee_Matrix;

            % For generating intermediate joints, NOT USED OUTSIDE OF
            % LAB2 TEST
            % t_trans_mat(:,:,6) = joint_1_2_Matrix;
            % t_trans_mat(:,:,7) = joint_2_3_Matrix;
            % t_trans_mat(:,:,8) = joint_3_4_Matrix;

            % t_trans_mat(:,:,9) = joint_4_ee_Matrix;

            % OPTIONAL TEST: checks if transformation matrix is valid
            %             determinent = round(det(t_trans_mat(:,:,5)),4);
            %             if determinent ~= 1
            %                 t_trans_mat = "not a valid transformation matrix! MAJOR ERROR!";
            %             end
        end

        % takes a full DH_Table and outputs the transformation matrix from
        % the base to the tip of the end effector symbolically 
        function t_trans_mat_syms = dh2fksym(self, DH_Table)
            self.joint_1_2_SymsMatrix = self.T_0_1 * self.dh2mat(DH_Table(1,:)); % matrix transformation from joint 1 to 2
            self.joint_1_3_SymsMatrix =  self.joint_1_2_SymsMatrix * self.dh2mat(DH_Table(2,:)); % matrix transformation from joint 2 to 3
            self.joint_1_4_SymsMatrix = self.joint_1_3_SymsMatrix * self.dh2mat(DH_Table(3,:)); % matrix transformation from joint 3 to 4
            self.joint_1_ee_SymsMatrix = self.joint_1_4_SymsMatrix * self.dh2mat(DH_Table(4,:)); % matrix transformation from joint 4 to end effector
            t_trans_mat_syms = self.joint_1_ee_SymsMatrix;
        end

        % FUNCTION task2ik()
        % Input:
        % 1×4 task space vector for x,y,z position (i.e. 𝑝 0 to ee) and
        % orientation of the end-effector w.r.t the horizontal plane (𝛼).
        %
        % Output:
        % 1 x 4 joint space vector for the joint variables q1,q1,q3,q4

        function joint_space = task2ik(self, task_space)
            % Define End Effector Point
            P_x = task_space(1);
            P_y = task_space(2);
            P_z = task_space(3);
            alpha = task_space(4);

            % Define Simple Variable Names for calculations
            L0 = self.mDim(1);
            L1 = self.mDim(2);
            L2 = self.mDim(3);
            L3 = self.mDim(4);
            L4 = self.mDim(5);

            % Solve for Theta 1 (q1) at Joint 1
            q1 = atan2d(P_y, P_x);

            % Define a point for end effector in plane R when arm is rotated called "r"
            r = sqrt(P_x^2 + P_y^2);

            % Define an in plane point "A" in plane R at joint 3
            A_r = r - cosd(alpha) * L4;
            A_z = (P_z - sind(alpha) * L4) - L0 - L1;
            LA = sqrt(A_r^2 + A_z^2); % distance from base to joint 3

            % Solve for theta 2 (q2) at joint 2
            b = atan2d(A_z, A_r);
            c = acosd((L2^2 + LA^2 - L3^2) / (2 * L2 * LA));
            q2 = self.L2Angle - c - b;

            % Solve for theta 3 (q3) at joint 3
            d = acosd((L3^2 + L2^2 - LA^2) / (2 * L3 * L2));
            q3 = d - 180 + self.L2Angle;
            q3=-q3;

            % Solve for theta 4 (q4)
            q4 = -alpha - q2 - q3;

            % Check if joints are in workspace
            self.isValidWorkspace([q1,q2,q3,q4]);
            joint_space(1) = round(q1,3);
            joint_space(2) = round(q2, 3);
            joint_space(3) = round(q3, 3);
            joint_space(4) = round(q4, 3);
            % Notes:
            % If the desired end-effector pose is unreachable, catch the exception and throw an error.
            % impose safety checks to ensure that the target position lies within the
            % reachable workspace of the robot arm – if that is not the case, the
            % function should throw an error using the error
        end


        % Inputs:
        % 1. Trajectory coefficients. This will be a 4x4 matrix of trajectory
        % coefficients (4 joints, 4 coefficients) which are generated from
        % cubic_traj() in the Traj_Planner.m class
        % 2. The total amount of time the trajectory should take. This must
        % be the same duration you passed to cubic_traj() to generate the
        % trajectories for each joint.
        %
        % Output:
        % Saved matrix data of time and joint angle data for plots
function trajectory = run_trajectory(self, traj_coefficients, trajectory_time, joint_or_task, model)
            % This function will run a while loop, which calculates the 
            % current joint poses based on the trajectory
            % coefficients and current time, and commands the robot to go 
            % to these with set_joint_vars()
            [~, columns] = size(traj_coefficients);
            tic; % start timer
            trajectory = [];
            % Loop running too fast leads to the arm missing signals
            % Only send signal every interval determined by refresh rate
            cur = 0;
            total_intervals = 0;
            
            refresh_rate = 20; % HZ

            plot_arm = false;
            if (exist("model", "var"))
                plot_arm = true;
                % Reduce refresh rate when live plotting, to reduce computi
                refresh_rate = 10; % HZ
            end
            
            interval = 1/refresh_rate; % calculate time between signals
            intervals_between_plots = 0;
            % loop every time interval

            while toc < trajectory_time
                curTime = toc;
                if curTime > cur
                    cur = cur + interval;
                    total_intervals = total_intervals + 1;
                    joint_vars = [0; 0; 0; 0];
                    for i = 1:4
                        if columns == 4
                            joint_vars(i, 1) = traj_coefficients(i, :) * [1; curTime; curTime^2; curTime^3];
                        else
                            joint_vars(i, 1) = traj_coefficients(i, :) * [1; toc; toc^2; toc^3; toc^4; toc^5];
                        end
                    end
    
                    % if the function is told to use task space, change
                    % calculated vars from x y z a to q1 q2 q3 q4
                    if joint_or_task == "task"
                        joint_vars = transpose(self.task2ik(joint_vars));
                    end
                    
                    % Move arm to joint vars.
                    % set joints if time has passed interval
                    self.set_joint_vars(joint_vars, interval);
                    if plot_arm && total_intervals > intervals_between_plots
                        total_intervals = 0;
                        % Collect velocities as well as joint variables to
                        % add to plot
                        readings = self.read_joint_vars(true, true);
                        model.update_plot(readings(1,:), readings(2, :));
                        trajectory = [trajectory;curTime, readings(1, :), readings(2,:)]; % append data
                    else
                        readings = self.read_joint_vars(true, false);
                        trajectory = [trajectory;curTime, readings(1,:)]; % append data
                    end
                    
                    
                end

            end % end trajectory while loop
        end % end trajectory method

        % INPUT: jointSpace is a 1x4 vector that contains the joint variables
        % OUTPUT: true if position is in valid workspace, if not throws an error
        % Notes:
        % If the desired end-effector pose is unreachable, catch the exception and throw an error.
        % impose safety checks to ensure that the target position lies within the
        % reachable workspace of the robot arm – if that is not the case, the
        % function should throw an error using the error

        function valid = isValidWorkspace(self, jointSpace)
            q1 = jointSpace(1);
            q2 = jointSpace(2);
            q3 = jointSpace(3);
            q4 = jointSpace(4);
            if q1 < -180 || q1 > 180
                error("Error: q1 is not in the workspace");
            elseif q2 < -90 || q2 > 90
                error("Error: q2 is not in the workspace");
            elseif q3 < -95 || q3 > 80
                error("Error: q3 is not in the workspace");
            elseif q4 < -90 || q4 > 100
                error("Error: q4 is not in the workspace");
            else
                valid = true;
            end
        end % End valid workspace function

        % Input position data, velocity date, acc data. output xyz data
        % for plots
        function [p_data, v_data, a_data] = jointTrajectory2xyz(self, joint_data)
            % convert to xyz
            [datapoints, ~] = size(joint_data);
            p_data = zeros(datapoints, 3);
            for i = 1:datapoints
                fk = self.joint2fk(joint_data(i, 2:5));
                p_data(i, :) = [fk(1, 4, 5), fk(2, 4, 5), fk(3, 4, 5)];
            end
            
            % find velocities
            v_data = diff(p_data)./diff(joint_data(:,1));
            
            % find accels
            a_data = diff(v_data)./diff(joint_data(1:end-1,1));
        end

        % get_jacobian_syms()
        % defines a jacobian matrix in terms of syms q1,q2,q3,q4
        function jacobian_sym = get_jacobian_syms(self)
            syms q1(t) q2(t) q3(t) q4(t) t
            fk = self.joint2fk([q1(t) q2(t) q3(t) q4(t)], true);
            pose = fk(1:3,4);
            Jp = jacobian(pose, [q1(t), q2(t), q3(t), q4(t)]) * 180/pi;
            J_omega = [
                0      self.joint_1_3_SymsMatrix(1,3)      self.joint_1_4_SymsMatrix(1,3)      self.joint_1_ee_SymsMatrix(1,3);
                0      self.joint_1_3_SymsMatrix(2,3)      self.joint_1_4_SymsMatrix(2,3)      self.joint_1_ee_SymsMatrix(2,3);
                1      self.joint_1_3_SymsMatrix(3,3)      self.joint_1_4_SymsMatrix(3,3)      self.joint_1_ee_SymsMatrix(3,3)
                ];
            jacobian_sym = [Jp ; J_omega];
        end

        % Inputs: 1x4 [q] containing all current joint angles
        % Outputs: Corresponding 6x4 Jacobian Matrix
        function J = get_jacobian(self, q)
            syms q1(t) q2(t) q3(t) q4(t) t
            J = eval(subs(self.jacobian_sym, [q1(t), q2(t), q3(t), q4(t)], q));
            self.jacobian_singularity(J);
        end

        % INPUT: J is the 6x4 jacobian of the current joint angles
        % OUTPUT: error when robot is too close to singularity, otherwise true
        % checks how close arm is to a singularity
        function not_singularity = jacobian_singularity(self, J)
            Jp = J(1:3, :);
            if abs(det(Jp(:, 1:3))) < 50
                error("Robot is too close to the arm singularity condition");
            else
                not_singularity = true;
            end
        end    

        % Inputs: q (4x1) and q_prime(4x1) representing the joint angles and
        % instantaneous joint velocities respectivily
        % Outputs: 6x1 vector of task-space linear velocities p_prime and
        % angular velocities omega
        function p_omega_velocities = vel2fdk(self, q, q_prime)
            jacobian = self.get_jacobian(q); % 6x4 jacobian matrix
            q_prime = [q_prime(1); q_prime(2) - self.velOffset(1); q_prime(3) - self.velOffset(2); q_prime(4)]; % stupid bug where joint vels are offset- current temp fix lol
            p_omega_velocities = jacobian * q_prime; % 6x1 containing linear velocities p_prime and angular velocities omega
        end


        % Inputs: q_0(1x4) current / initial joint posiitons. desired task space (6x1), task_space
        % Outputs: iterations to desired task space
        function q_i = NewtonRaphsonIK(self, q_0, task_space)
            Kp = 1.75;
            iterations = 0;
            % Define tolderance for algo
            epsilon = 5;
            q_i = q_0; % start point
            fk_all = self.joint2fk(q_i);
            model = Model(self);


            current_task_space = fk_all(1:3,4,5);

            % Convert from rot mat to Euler Angles
            current_rot_mat = fk_all(1:3,1:3,5);
            EulAng = rad2deg(rotm2eul(current_rot_mat))';
            current_task_space = [current_task_space; EulAng];

            while norm(task_space - current_task_space) > epsilon
                disp(norm(task_space - current_task_space));
                fk_all = self.joint2fk(q_i);

                % obtain task space and rotation for Jp and Jw
                current_task_space = fk_all(1:3,4,5);
                current_rot_mat = fk_all(1:3,1:3,5);

                % Convert from rot mat to Euler Angles
                EulAng = rad2deg(rotm2eul(current_rot_mat))';
                current_task_space = [current_task_space; EulAng];

                J = self.get_jacobian(q_i);

                delta_q = pinv(J) * Kp * (task_space - current_task_space); % should output changes to joint variables needed
                q_i = q_i + delta_q';
                model.plot_arm(q_i);
                iterations = iterations + 1;
            end
            pause(2);
            disp('Iterations: ');
            disp(iterations);
        end

        % INPUT: ball_loc is x and y in task space, pick_or_place
        % is a string that determines whether the gripper picks up or puts down
        function pick_and_place(self, ball_loc, pick_or_place)
            self.go_to_xyzalpha([ball_loc, 70, -85], "joint");
            if pick_or_place == "pick"
                self.go_to_xyzalpha([ball_loc, 23, -85], "task");
                self.writeGripper(false);
            else
                self.go_to_xyzalpha([ball_loc, 5, -85], "task");
                self.writeGripper(true);
            end    
            self.go_to_xyzalpha([ball_loc, 70, -85], "task");
            

        end 
        
        % INPUT: target_pose is an 1x4 array that indicates location in
        % task space. task_or_joint is a string that indicates whether the
        % trajectory is in task space or joint space
        function go_to_xyzalpha(self, target_pose, task_or_joint)
            % Choose travel time of go to function here
            travel_time = 1;

            % Utilize trajectory generation class
            traj_planner = Traj_Planner();

            % Find Starting position
            readings = self.read_joint_vars(true, false);
            
            coefficients = zeros(4, 6);
            % If task space trajectory, convert joint readings to task
            % space position using FK, find task trajectory between current task
            % space position and target position
            if task_or_joint == "task"
                fk = self.joint2fk(readings(1,:));
                alpha = -1* (readings(1,2) + readings(1,3) + readings(1,4));
                starting_poses = [fk(1, 4, 5), fk(2, 4, 5), fk(3, 4, 5), alpha];
            % If joint space trajectory, convert target position into joint
            % space using IK. Find joint space trajectory between current joint
            % space position and target joint space position
            else 
                starting_poses = readings(1,:);
                target_pose = self.task2ik(target_pose);
            end    
            for i = 1:4
                coefficients(i, :) = traj_planner.quintic_traj(0, travel_time, 0, 0, 0, 0, starting_poses(i), target_pose(i));
            end    
            self.run_trajectory(coefficients, travel_time, task_or_joint);
        end

        % Takes an input of a location on the undistorted image and a camera and outputs
        % the world coords for that point
        % also adjust coordinates based on height of balls
        function WorldFramePoint = ball2world(self, coordinates)
            checkPoints = pointsToWorld(self.cam.cam_IS,self.cam.cam_R,self.cam.cam_T,coordinates);
            ydist = self.cam2check - checkPoints(2);
            deltay = ydist / self.camheight * self.ballradius;
            checkPoints(2) = checkPoints(2) + deltay;
            WorldFramePoint = self.T_checker_0 * [checkPoints'; 0; 1];
        end


        % Takes an input of a location on the undistorted image and a camera and outputs
        % the world coords for that point
        % does not adjust coordinates
        function WorldFramePoint = object2world(self, coordinates)
            checkPoints = pointsToWorld(self.cam.cam_IS,self.cam.cam_R,self.cam.cam_T,coordinates);
            WorldFramePoint = self.T_checker_0 * [checkPoints'; 0; 1];
        end

        % Get centroids of ball of given color given a cam
        % Accepts 'red', 'orange', 'yellow', 'green', 'gray'
        % returns centroids in world frame
        function centroid = getCentroids(self, color)
            if color == "object"
                load("checkerboard.mat"); % base img
                img = self.cam.getCroppedImage(); % current image
                diff = imfuse(baseimg,img, "diff"); % difference between img and baseimg
                mediandiff = medfilt2(diff); % medianfiltered image
                binarymask = im2bw(mediandiff, 0.6); % binary mask
                binarymask = medfilt2(binarymask); % medianfiltered mask
                mask = imclose(binarymask, 20); % dialation and erosion mask
                
                % plot image processing pipeline
                subplot(2,3,1), imshow(img);
                title("Current Image");
                subplot(2,3,2), imshow(diff);
                title("Diffrence between base and current");
                subplot(2,3,3), imshow(mediandiff); 
                title("median filtered difference");
                subplot(2,3,4), imshow(binarymask);
                title("binary mask");
                subplot(2,3,5), imshow(mask);
                title("post dialation and erosion");
            else
                img = self.cam.getCroppedImage(); % base cropped image
                binarymask = self.cam.colorMask(img, color); % color mask
                binarymask = medfilt2(binarymask); % medianfiltered mask
                mask = imclose(binarymask, 20); % dialation and erosion
                
                % plot image processing pipeline
                subplot(2,3,1), imshow(img);
                title("Current Image");
                subplot(2,3,4), imshow(binarymask);
                title("binary mask");
                subplot(2,3,5), imshow(mask);
                title("post dialation and erosion");
            end
            drawnow;

            blobAnalysis = vision.BlobAnalysis(AreaOutputPort = true,...
                CentroidOutputPort = false,...
                BoundingBoxOutputPort = true,...
                MinimumBlobArea = 75, ExcludeBorderBlobs = true);
            [areas, boxes] = step(blobAnalysis, mask);

            % Sort connected components in descending order by area
            [~, idx] = sort(areas, "Descend");

            % Get the two largest components.
            boxes = double(boxes(idx, :));
            
            [detected_points, ~] = size(boxes);
            centroid = [-1; -1; -1; -1];

            if detected_points > 0
                for i = 1:detected_points
                    % check to see if area is ball size- if its grey, it
                    % sees random connected stuff that can be really big
                    if color == "object"
                        % Get the top-left and the top-right corners.
                        box = double(boxes(1, :));
                        % Compute the center 
                        centroid_pixel = box(1:2) + box(3:4)/2;
                        centroid = self.object2world(centroid_pixel);
                    % Check to see if area of boxes is reasonable for ball
                    elseif (3800 > boxes(1, 3) * boxes(1, 4) && boxes(1, 3) * boxes(1, 4) > 1400)
                        % Get the top-left and the top-right corners.
                        box = double(boxes(1, :));
                        % Compute the center
                        centroid_pixel = box(1:2) + box(3:4)/2;
                        centroid = self.ball2world(centroid_pixel);
                    end
                end
            end 
        end % end get centroids           
        
        % Input: q is a 1x4 array indicating joint positions of robot
        % Output: A is the manipulability Elipsoid of the arm at a given
        % position.
        function A = manipulability_elipsoid(self, q)
            jacobian = self.get_jacobian(q);
            j_p = jacobian(1:3, :);
            j_p_transpose = transpose(j_p);
            A = j_p * j_p_transpose;
        end
        
        % Takes picture and saves it with name baseimg
        function getBaseImg(self)
            baseimg = self.cam.getCroppedImage();
            save("checkerboard.mat","baseimg");
        end

        % Output: moving is a true or false boolean, is true when motion is
        % detected in the previous 0.1 seconds.
        function moving = checkMovement(self)
            % Saves initial image
            prevImg = self.cam.getCroppedImage();

            % Waits 0.1 seconds and takes another image
            pause(0.1);
            curImg = self.cam.getCroppedImage();

            % Display
            subplot(2,3,6), imshow(curImg);
            title("live feed")
            drawnow;
            
            % finds difference between current image and previous image
            diff = imfuse(curImg,prevImg,"diff");

            % Image enhancement reduces noisie
            mediandiff = medfilt2(diff, [5,5]);

            % Segmentation, binarize the image
            binaryImage = im2bw(mediandiff, 0.5);

            % Information extraction, pixels that have changed are now
            % white, 1, and pixels that did not change are black, 0
            % Summing total of image finds number of pixels that have
            % changed.
            numWhitePixels = sum(binaryImage(:));
            if numWhitePixels > 50
                moving = true;
            else
                moving = false;
            end
            % DEBUGGING Line: disp(moving);
        end

    end % end methods
end % end class
