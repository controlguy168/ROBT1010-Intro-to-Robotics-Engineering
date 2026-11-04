function redObjectTrackerUI()
    % ELEGOO Smart Robot Car - Red Object Detection & Size/Center Tracker UI
    % Connects using the provided ElegooRobotCar class framework.

    % --- 1. Connection Setup ---
    hostIP = '192.168.4.1'; 
    try
        robot = ElegooRobotCar(hostIP);
        robot.stop();
        robot.setWatchdog(1000);
        cleanupObj = onCleanup(@() safeClose(robot));
    catch ME
        uialert(uifigure, ['Failed to connect to robot at ', hostIP, '. Check connection. Error: ', ME.message], 'Connection Error');
        return;
    end

    isRoutineRunning = false;

    % --- 2. Create Main UI Figure & Layout ---
    fig = uifigure('Name', 'Elegoo Robot Car - Red Object Tracker', 'Position', [50, 50, 1250, 750]);
    
    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {'3x', '2x'};

    leftGrid = uigridlayout(mainGrid, [2, 1]);
    leftGrid.RowHeight = {'2.5x', '1.5x'};

    rightGrid = uigridlayout(mainGrid, [2, 1]);
    rightGrid.RowHeight = {'2.5x', '2.5x'};

    % ==================== VIDEO FEED PANEL ====================
    videoPanel = uipanel(leftGrid, 'Title', 'Live Red Object Tracking Feed');
    videoGrid = uigridlayout(videoPanel, [1, 1]);
    axVideo = uiaxes(videoGrid);
    axVideo.XTick = []; axVideo.YTick = [];
    title(axVideo, 'Camera View');

    % ==================== CONTROL & STATUS PANEL ====================
    controlPanel = uipanel(leftGrid, 'Title', 'Tracker Controls');
    controlGrid = uigridlayout(controlPanel, [5, 2]);

    uilabel(controlGrid, 'Text', 'Drive Speed v:');
    speedInput = uieditfield(controlGrid, 'numeric', 'Value', 50, 'Limits', [10, 255]);

    uilabel(controlGrid, 'Text', 'Turn Speed omega (Default 50):');
    omegaInput = uieditfield(controlGrid, 'numeric', 'Value', 50, 'Limits', [10, 255]);

    startBtn = uibutton(controlGrid, 'Text', 'Find & Track Red Object', ...
        'BackgroundColor', [0.4 0.8 0.4], ...
        'ButtonPushedFcn', @(~,~) runRedTracker(speedInput.Value, omegaInput.Value));
    startBtn.Layout.Column = [1, 2];

    stopBtn = uibutton(controlGrid, 'Text', 'STOP TRACKER', ...
        'BackgroundColor', [1 0.6 0.6], ...
        'ButtonPushedFcn', @(~,~) stopRoutine());
    stopBtn.Layout.Column = [1, 2];

    % ==================== RIGHT PANEL: PLOT & LOGS ====================
    rightTopPanel = uipanel(rightGrid, 'Title', 'Target Size Ratio');
    axSizePlot = uiaxes(uigridlayout(rightTopPanel, [1, 1]));
    title(axSizePlot, 'Current Area vs Initial Target Area');
    xlabel(axSizePlot, 'Sample'); ylabel(axSizePlot, 'Area Ratio');

    rightBottomPanel = uipanel(rightGrid, 'Title', 'System Log');
    statusTextArea = uitextarea(uigridlayout(rightBottomPanel, [1, 1]), 'Editable', 'off', ...
        'Value', ["System Ready.", "Place a red object in view and click 'Find & Track Red Object'."]);

    maxPoints = 40;
    timeBuffer = zeros(1, maxPoints);
    sizeBuffer = zeros(1, maxPoints);
    
    fig.CloseRequestFcn = @(~,~) closeUI(fig, robot);

    function logMsg(msg)
        currentText = string(statusTextArea.Value);
        timestampedMsg = "[" + datestr(now, 'HH:MM:SS') + "] " + msg;
        statusTextArea.Value = [timestampedMsg; currentText];
        drawnow limitrate;
    end

    function runRedTracker(speed, turnSpd)
        if isRoutineRunning
            return;
        end

        logMsg('Scanning camera frame for a red object...');
        
        % 1. Capture initial frame to find the red object
        initialFrame = [];
        try
            initialFrame = robot.capture();
        catch ME
            logMsg(['Failed to capture initial frame: ', ME.message]);
            return;
        end

        if isempty(initialFrame)
            logMsg('Error: Captured frame is empty.');
            return;
        end

        % Process Red Mask on initial frame to confirm target
        [initialArea, initialBbox, foundValidRed] = detectRedBlob(initialFrame);

        if ~foundValidRed
            uialert(fig, 'No prominent red object found in view. Please reposition the red object and try again.', 'Red Object Not Found');
            logMsg('Error: Red object detection failed. Target not found.');
            return;
        end

        logMsg(['SUCCESS: Red object confirmed and locked! Initial Area = ', num2str(initialArea), ' px²']);
        
        isRoutineRunning = true;
        startTime = tic;
        timeBuffer(:) = 0;
        sizeBuffer(:) = 0;
        lostFrameCount = 0;

        try
            while isRoutineRunning
                frame = [];
                try
                    frame = robot.capture();
                catch
                end

                if isempty(frame)
                    continue;
                end

                [imgHeight, imgWidth, ~] = size(frame);
                centerX = imgWidth / 2;

                % Detect Red Blob in current frame
                [currArea, bbox, targetFound] = detectRedBlob(frame);

                errorX = 0;
                areaRatio = 1.0;

                if targetFound
                    lostFrameCount = 0;
                    centroidX = bbox(1) + (bbox(3) / 2);
                    centroidY = bbox(2) + (bbox(4) / 2);
                    
                    errorX = centroidX - centerX;
                    areaRatio = currArea / initialArea;

                    % Update size plot buffer
                    timeBuffer = [timeBuffer(2:end), toc(startTime)];
                    sizeBuffer = [sizeBuffer(2:end), areaRatio];
                    plot(axSizePlot, timeBuffer, sizeBuffer, 'r-', [timeBuffer(1), timeBuffer(end)], [1, 1], 'k--');
                    legend(axSizePlot, {'Area Ratio', 'Target Size (1.0)'}, 'Location', 'northwest');

                    % Draw visual bounding box and center marker
                    annotatedFrame = insertShape(frame, 'Rectangle', bbox, 'Color', 'green', 'LineWidth', 3);
                    annotatedFrame = insertMarker(annotatedFrame, [centroidX, centroidY], 'x', 'Color', 'yellow', 'Size', 10);
                    imshow(annotatedFrame, 'Parent', axVideo);
                else
                    lostFrameCount = lostFrameCount + 1;
                    imshow(frame, 'Parent', axVideo);
                end

                % 2. Control Logic with Deadzones
                if lostFrameCount > 8
                    robot.stop();
                    logMsg('Red target lost. Stopping and waiting...');
                else
                    % Deadzone thresholds
                    steerDeadzone = 35;  % Pixels from center
                    sizeMin = 0.80;      % Lower distance bound
                    sizeMax = 1.20;      % Upper distance bound

                    % A. Horizontal Centering (Steering with Deadzone)
                    if abs(errorX) > steerDeadzone
                        if errorX > 0
                            robot.turnRight(turnSpd);
                        else
                            robot.turnLeft(turnSpd);
                        end
                    else
                        % B. Size / Distance Maintenance (Linear Motion with Deadzone)
                        if areaRatio < sizeMin
                            % Target too small / far -> Move forward
                            robot.forward(speed);
                        elseif areaRatio > sizeMax
                            % Target too big / close -> Back up
                            robot.backward(speed);
                        else
                            % Inside deadzone (Optimal size & centered) -> Hold position
                            robot.stop();
                        end
                    end
                end

                pause(0.04);
            end

            robot.stop();
            logMsg('Red object tracking routine stopped.');

        catch ME
            logMsg(['Routine error: ', ME.message]);
            robot.stop();
            isRoutineRunning = false;
        end
    end

    function [area, bbox, found] = detectRedBlob(frame)
        area = 0;
        bbox = [0 0 0 0];
        found = false;

        try
            hsvImg = rgb2hsv(frame);
            H = hsvImg(:,:,1);
            S = hsvImg(:,:,2);
            V = hsvImg(:,:,3);

            % Red color wraps around 0 and 1 in HSV hue space
            redMask = ((H < 0.08 | H > 0.92) & S > 0.35 & V > 0.25);
            
            % Clean up noise
            redMask = bwareaopen(redMask, 200);
            se = strel('disk', 4);
            redMask = imclose(redMask, se);

            stats = regionprops(redMask, 'Area', 'BoundingBox');
            if ~isempty(stats)
                [maxArea, maxIdx] = max([stats.Area]);
                if maxArea > 300 % Minimum pixel threshold for valid red target
                    area = maxArea;
                    bbox = stats(maxIdx).BoundingBox;
                    found = true;
                end
            end
        catch
        end
    end

    function stopRoutine()
        isRoutineRunning = false;
        try
            robot.stop();
            logMsg('Routine stopped by user.');
        catch
        end
    end
end

function safeClose(robot)
    try
        robot.stop();
        robot.close();
    catch
    end
end

function closeUI(fig, robot)
    try
        safeClose(robot);
    catch
    end
    delete(fig);
end
