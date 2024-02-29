classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.

    properties
        % Properties
        params;     % Camera Parameters
        cam;        % Webcam Object
        cam_pose;   % Camera Pose (transformation matrix)
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
        cam_TForm;   % Camera Rigid 3D TForm
        topLine;    % Top most position of Mask
        bottomLine; % Bottom most position of mask
        leftColumn; % Left most position of mask
        rightColumn; % Right most position of mask
    end

    methods

        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();

            % Set mask defaults
            self.topLine = 209;
            self.bottomLine = 880;
            self.leftColumn = 130;
            self.rightColumn = 349;
        end

        % Getter Methods
        function tForm = getTForm(self)
            tForm = self.cam_TForm;
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraIntrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end
        
        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end

        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end
        end

        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end


        function [newIs, pose] = calculateCameraPos(self)  % DO NOT USE
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-1)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            self.cam_TForm = rigid3d([ self.cam_R, zeros(3,1); self.cam_T, 1 ]);

            pose = [   R,    t';
                0, 0, 0, 1];
        end

        function croppedImage = getCroppedImage(self)
            firstImage = self.getImage();
%             imshow(firstImage);
            try
                load("binaryImage.mat");
%                 disp("Loaded binary mask");
            catch exception
                disp("Could not find binary mask");

                fontSize = 16;
                imshow(firstImage, []);
                hFH = drawpolygon;
                % Create a binary image ("mask") from the ROI object.
                binaryImage = hFH.createMask();

                % Get coordinates of the boundary of the freehand drawn region.
                structBoundaries = bwboundaries(binaryImage);
                polygonxy =structBoundaries{1}; % Get n by 2 array of x,y coordinates.
                x = polygonxy(:, 2); % Columns.
                y = polygonxy(:, 1); % Rows.

                self.topLine = min(x);
                self.bottomLine = max(x);
                self.leftColumn = min(y);
                self.rightColumn = max(y);

                save("binaryImage.mat","binaryImage");
                disp("Saved binary mask to binaryImage.mat");
            end
            % Mask the image and display it.
            % Will keep only the part of the image that's inside the mask, zero outside mask.
            blackMaskedImage = firstImage;
            % use the mask for each of the RGB values of the blackMaskedImage.
            for i = 1:3
                colorIndex = blackMaskedImage(:,:,i);
                colorIndex(~binaryImage) = 0;              % sets the color outside the mask to a value of 0.
                blackMaskedImage(:,:,i) = colorIndex;
            end
            croppedImage = blackMaskedImage;
        end

        % Selects color mask based on ball type input
        function [BW,maskedRGBImage] = colorMask(self, RGB, ball)
            switch ball
                case 'red'
                    [BW,maskedRGBImage] = self.redMask(RGB);
                case 'green'
                    [BW,maskedRGBImage] = self.greenMask(RGB);
                case 'yellow'
                    [BW,maskedRGBImage] = self.yellowMask(RGB);
                case 'orange'
                    [BW,maskedRGBImage] = self.orangeMask(RGB);
                case 'gray'
                    [BW,maskedRGBImage] = self.grayMask(RGB);
                otherwise
                    error("Please color your balls");
            end
        end


    %% AUTO GENERATED MASK FUNCTIONS
    % INPUT: RGB Image
    % Output: BW, binary mask based on colors
    % maskedRGBImage, input image with mask applied
        function [BW,maskedRGBImage] = grayMask(self, RGB)
            %createMask  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 26-Feb-2024
            %------------------------------------------------------


            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.000;
            channel1Max = 1.000;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.000;
            channel2Max = 0.184;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.383;
            channel3Max = 0.691;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
        end

        function [BW,maskedRGBImage] = redMask(self, RGB)
            %createMask  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 19-Feb-2024
            %------------------------------------------------------


            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.960;
            channel1Max = 0.057;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.159;
            channel2Max = 0.828;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.343;
            channel3Max = 1.000;

            % Create mask based on chosen histogram thresholds
            sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

        end

        function [BW,maskedRGBImage] = greenMask(self, RGB)
            %createMask  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 20-Feb-2024
            %------------------------------------------------------


            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.371;
            channel1Max = 0.479;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.271;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.090;
            channel3Max = 0.798;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
        end

        function [BW,maskedRGBImage] = yellowMask(self, RGB)
            %createMask  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 19-Feb-2024
            %------------------------------------------------------


            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.111;
            channel1Max = 0.215;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.397;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.479;
            channel3Max = 1.000;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

        end

        function [BW,maskedRGBImage] = orangeMask(~, RGB)
            %createMask  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 19-Feb-2024
            %------------------------------------------------------


            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.056;
            channel1Max = 0.115;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.397;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.479;
            channel3Max = 1.000;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

        end
    end
end