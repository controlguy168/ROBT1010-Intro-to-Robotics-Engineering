function elegooCarDashboard()
    % ELEGOO Smart Robot Car Dashboard & Control UI
    % Features:
    % - Default turning speed: 60
    % - Default pulse control time interval: 0.1 s
    % - Collision Avoidance & Automatic 180° Sweep Scan when distance <= 250mm
    % - Configurable Stream Rate (FPS control) & AVI Recording with non-blocking sensor protection
    % - Full Teleoperation & Sensor Monitoring

    % --- 1. Connection Setup ---
    hostIP = '192.168.4.1'; % Default AP mode IP for Elegoo ESP32
    try
        robot = ElegooRobotCar(hostIP);
        robot.stop();
        robot.setWatchdog(1000);
        cleanupObj = onCleanup(@() safeClose(robot));
    catch ME
        uialert(uifigure, ['Failed to connect to robot at ', hostIP, '. Check connection. Error: ', ME.message], 'Connection Error');
        return;
    end

    % State flags & Video Recording Variables
    sweepActive = false;
    isDriving = false;
    isHandlingObstacle = false;
    
    isRecording = false;
    videoWriterObj = [];

    % --- 2. Create Main UI Figure & Layout ---
    fig = uifigure('Name', 'Elegoo Robot Car Control & Sensor Dashboard (Robust Stream & Rec)', ...
                   'Position', [30, 30, 1350, 900]);
    
    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {'4.5x', '2x'};

    leftGrid = uigridlayout(mainGrid, [2, 1]);
    leftGrid.RowHeight = {'2.8x', '3.5x'};

    rightGrid = uigridlayout(mainGrid, [2, 1]);
    rightGrid.RowHeight = {'2.5x', '2.5x'};

    % ==================== SENSOR VISUALIZATION PANEL ====================
    sensorPanel = uipanel(leftGrid, 'Title', 'Real-Time Sensor Readings & Live Camera Stream');
    sensorGrid = uigridlayout(sensorPanel, [1, 4]);

    axIMU = uiaxes(sensorGrid); title(axIMU, 'Gyro (Z) & Accel (Y)');
    xlabel(axIMU, 'Sample'); ylabel(axIMU, 'Val');
    
    axRange = uiaxes(sensorGrid); title(axRange, 'Ultrasonic Range');
    xlabel(axRange, 'Sample'); ylabel(axRange, 'Distance (cm)');
    
    axLine = uiaxes(sensorGrid); title(axLine, 'Line Track (IR)');
    xlabel(axLine, 'Sample'); ylabel(axLine, 'Digital');
    
    axCam = uiaxes(sensorGrid); title(axCam, 'Live Video Stream');
    axis(axCam, 'off');

    % ==================== DRIVE CONTROL PANEL ====================
    drivePanel = uipanel(leftGrid, 'Title', 'Drive Control & Teleoperation');
    driveGrid = uigridlayout(drivePanel, [1, 2]);
    driveGrid.ColumnWidth = {'1x', '1.2x'};

    paramGrid = uigridlayout(driveGrid, [4, 2]);
    paramGrid.RowHeight = {'1x', '1x', '1x', '1.5x'};
    paramGrid.ColumnWidth = {'2.2x', '1x'};
    
    uilabel(paramGrid, 'Text', 'Linear Speed v (-255 to 255):');
    vInput = uieditfield(paramGrid, 'numeric', 'Value', 100, 'Limits', [-255, 255]);

    uilabel(paramGrid, 'Text', 'Angular Speed omega (-255 to 255):');
    omegaInput = uieditfield(paramGrid, 'numeric', 'Value', 60, 'Limits', [-255, 255]); % Default 60

    uilabel(paramGrid, 'Text', 'Duration T (seconds):');
    tInput = uieditfield(paramGrid, 'numeric', 'Value', 3, 'Limits', [0.1, 60]);

    runTimedBtn = uibutton(paramGrid, 'Text', 'Run Timed Drive (v, \omega, T)', ...
        'BackgroundColor', [0.6 0.8 1], ...
        'ButtonPushedFcn', @(~,~) executeTimedDrive());
    runTimedBtn.Layout.Column = [1, 2];
    
    teleopGrid = uigridlayout(driveGrid, [4, 3]);
    teleopGrid.RowHeight = {'1.2x', '1.2x', '1.2x', '0.8x'};

    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);
    btnForward = uibutton(teleopGrid, 'Text', '▲ Forward', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.forward(vInput.Value)));
    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);

    btnLeft = uibutton(teleopGrid, 'Text', '◄ Turn Left', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.turnLeft(omegaInput.Value)));
    btnStop = uibutton(teleopGrid, 'Text', '■ STOP', 'BackgroundColor', [1 0.6 0.6], ...
        'ButtonPushedFcn', @(~,~) stopDrive());
    btnRight = uibutton(teleopGrid, 'Text', 'Turn Right ►', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.turnRight(omegaInput.Value)));

    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);
    btnBackward = uibutton(teleopGrid, 'Text', '▼ Backward', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.backward(vInput.Value)));
    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);

    uilabel(teleopGrid, 'Text', 'Pulse Time (s):');
    pulseInput = uieditfield(teleopGrid, 'numeric', 'Value', 0.1, 'Limits', [0.01, 5.0]); % Default 0.1s
    pulseInput.Layout.Column = [2, 3];

    % ==================== SCANNING & STREAMING CONFIGURATION PANEL ====================
    scanPanel = uipanel(rightGrid, 'Title', 'Scanning & Stream Rate Config');
    scanGrid = uigridlayout(scanPanel, [4, 2]);

    uilabel(scanGrid, 'Text', 'Scan Mode:');
    scanModeDropdown = uidropdown(scanGrid, 'Items', {'Fixed Angle', 'Continuous Sweep'}, ...
        'ValueChangedFcn', @(src, ~) toggleScanMode(src.Value));

    uilabel(scanGrid, 'Text', 'Pan Angle (0-180 deg):');
    angleSpinner = uispinner(scanGrid, 'Value', 90, 'Limits', [0, 180], ...
        'ValueChangedFcn', @(src, ~) applyFixedAngle(angleSpinner.Value));

    uilabel(scanGrid, 'Text', 'Stream Rate (FPS):');
    streamRateDropdown = uidropdown(scanGrid, 'Items', {'2 FPS (High Performance)', '5 FPS (Balanced)', '10 FPS (High Quality)', 'Full Rate'}, ...
        'Value', '5 FPS (Balanced)', ...
        'ValueChangedFcn', @(~,~) updateStreamTimerPeriod());

    applyScanBtn = uibutton(scanGrid, 'Text', 'Apply Fixed Angle', ...
        'ButtonPushedFcn', @(~,~) applyFixedAngle(angleSpinner.Value));
    applyScanBtn.Layout.Column = [1, 2];

    % ==================== DATA & SNAPSHOT WORKSPACE PANEL ====================
    dataPanel = uipanel(rightGrid, 'Title', 'Data Logging & Video Recording');
    dataGrid = uigridlayout(dataPanel, [4, 1]);

    uibutton(dataGrid, 'Text', 'Capture Camera Snapshot', ...
        'ButtonPushedFcn', @(~,~) captureAndShowSnapshot());

    btnVideoRec = uibutton(dataGrid, 'Text', 'START Video Recording', ...
        'BackgroundColor', [0.3 0.8 0.3], 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @(~,~) toggleVideoRecording());

    uibutton(dataGrid, 'Text', 'Save Current Drive Data to Workspace', ...
        'ButtonPushedFcn', @(~,~) saveWorkspaceData());

    uibutton(dataGrid, 'Text', 'Take UI Window Snapshot (PNG)', ...
        'ButtonPushedFcn', @(~,~) takeUISnapshot(fig));

    % --- Data Buffers for Real-Time Plotting ---
    maxPoints = 50;
    timeBuffer = zeros(1, maxPoints);
    gyroBuffer = zeros(1, maxPoints);
    accelYBuffer = zeros(1, maxPoints);
    rangeBuffer = zeros(1, maxPoints);
    lineBuffer  = zeros(1, maxPoints);
    
    % --- Background Polling Timer ---
    sensorTimer = timer('Period', 0.2, 'ExecutionMode', 'fixedRate', ...
        'TimerFcn', @(~,~) updateSensorsAndStream());
    start(sensorTimer);

    fig.CloseRequestFcn = @(~,~) closeUI(fig, sensorTimer, robot);

    % --- Nested Callback Functions ---
    function updateStreamTimerPeriod()
        stop(sensorTimer);
        switch streamRateDropdown.Value
            case '2 FPS (High Performance)'
                sensorTimer.Period = 0.5;
            case '5 FPS (Balanced)'
                sensorTimer.Period = 0.2;
            case '10 FPS (High Quality)'
                sensorTimer.Period = 0.1;
            case 'Full Rate'
                sensorTimer.Period = 0.05;
        end
        start(sensorTimer);
    end

    function updateSensorsAndStream()
        % 1. Read Sensors Safely with Individual Try-Catch Blocks
        gVal = 0; ayVal = 0;
        try
            imuData = robot.readIMU();
            if isstruct(imuData)
                if isfield(imuData, 'gyro'), gVal = imuData.gyro(3); end
                if isfield(imuData, 'accel'), ayVal = imuData.accel(2); end
            end
        catch
        end

        dVal = 0;
        try
            distData = robot.readDistance();
            if isstruct(distData) && isfield(distData, 'distance')
                dVal = double(distData.distance);
            end
        catch
        end

        lVal = 0;
        try
            irData = robot.readIR();
            if isstruct(irData)
                if isfield(irData, 'State')
                    lVal = double(irData.State);
                elseif isfield(irData, 'IR')
                    lVal = double(sum(irData.IR));
                end
            end
        catch
        end

        % 2. Collision Avoidance & Automatic 180° Servo Scan (Threshold: 250 mm = 25 cm)
        thresholdCm = 50.0; 
        if dVal > 0 && dVal <= thresholdCm
            if ~isHandlingObstacle
                isHandlingObstacle = true;
                isDriving = false;
                try
                    robot.stop();
                    robot.backward(100);
                    pause(0.5);
                    robot.stop();
                    performObstacleFull180Scan();
                catch
                end
            end
        else
            isHandlingObstacle = false;
        end

        % 3. Update Plots Safely
        try
            timeBuffer = [timeBuffer(2:end), toc];
            gyroBuffer = [gyroBuffer(2:end), gVal];
            accelYBuffer = [accelYBuffer(2:end), ayVal];
            rangeBuffer = [rangeBuffer(2:end), dVal];
            lineBuffer  = [lineBuffer(2:end), lVal];

            plot(axIMU, timeBuffer, gyroBuffer, 'r-', timeBuffer, accelYBuffer, 'b--');
            legend(axIMU, {'Gyro Z', 'Accel Y'}, 'Location', 'northwest');
            
            plot(axRange, timeBuffer, rangeBuffer, 'b-');
            plot(axLine, timeBuffer, lineBuffer, 'g-');
        catch
        end

        % 4. Live Video Streaming & Recording Safely
        try
            frame = robot.capture();
            if ~isempty(frame)
                imshow(frame, 'Parent', axCam);
                if isRecording && ~isempty(videoWriterObj)
                    writeVideo(videoWriterObj, frame);
                end
            end
        catch
            % If frame capture times out or drops, recording is safely terminated to prevent lockup
            if isRecording
                toggleVideoRecording();
                uialert(fig, 'Video recording auto-stopped due to a stream timeout or network drop.', 'Recording Warning');
            end
        end
    end

    function performObstacleFull180Scan()
        scanAngles = [0, 30, 60, 90, 120, 150, 180, 90];
        for k = 1:length(scanAngles)
            try
                robot.pan(scanAngles(k));
                pause(0.15);
            catch
            end
        end
    end

    function toggleVideoRecording()
        if ~isRecording
            filename = ['ElegooVideo_', datestr(now, 'yyyymmdd_HHMMSS'), '.avi'];
            try
                videoWriterObj = VideoWriter(filename, 'Uncompressed AVI');
                videoWriterObj.FrameRate = max(1, round(1 / sensorTimer.Period));
                open(videoWriterObj);
                isRecording = true;
                btnVideoRec.Text = 'STOP Video Recording';
                btnVideoRec.BackgroundColor = [0.9 0.2 0.2];
                uialert(fig, ['Recording started. Saving to ', filename], 'Recording Started');
            catch ME
                uialert(fig, ['Failed to start video recording: ', ME.message], 'Recording Error');
                videoWriterObj = [];
                isRecording = false;
            end
        else
            isRecording = false;
            btnVideoRec.Text = 'START Video Recording';
            btnVideoRec.BackgroundColor = [0.3 0.8 0.3];
            try
                if ~isempty(videoWriterObj)
                    close(videoWriterObj);
                end
            catch
            end
            videoWriterObj = [];
            uialert(fig, 'Video recording stopped and file saved successfully.', 'Recording Saved');
        end
    end

    function executeTimedDrive()
        v = vInput.Value;
        omega = omegaInput.Value;
        T = tInput.Value;
        
        timerObj = tic;
        while toc(timerObj) < T
            if isHandlingObstacle, break; end
            try
                robot.forward(v);
                if omega ~= 0
                    robot.drive(max(min(v + omega, 255), -255), max(min(v - omega, 255), -255));
                end
            catch
            end
            pause(0.02);
        end
        try, robot.stop(); catch, end
    end

    function startContinuousDrive(driveFunc)
        if isHandlingObstacle, return; end
        isDriving = true;
        pTime = pulseInput.Value;
        try
            driveFunc();
        catch
        end
        
        startTime = tic;
        while isDriving && ~isHandlingObstacle && toc(startTime) < (pTime + 0.1)
            pause(0.02);
        end
        if ~isDriving || isHandlingObstacle
            try, robot.stop(); catch, end
        end
    end

    function stopDrive()
        isDriving = false;
        try, robot.stop(); catch, end
    end

    function toggleScanMode(modeStr)
        if strcmp(modeStr, 'Continuous Sweep')
            sweepActive = true;
            scanModeDropdown.Value = 'Continuous Sweep';
            angles = [0, 30, 60, 90, 120, 150, 180, 150, 120, 90, 60, 30];
            idx = 1;
            while sweepActive && isvalid(fig)
                try
                    robot.pan(angles(idx));
                catch
                end
                idx = mod(idx, length(angles)) + 1;
                pause(0.25);
            end
        else
            sweepActive = false;
        end
    end

    function applyFixedAngle(angleVal)
        sweepActive = false;
        scanModeDropdown.Value = 'Fixed Angle';
        try, robot.pan(angleVal); catch, end
    end

    function captureAndShowSnapshot()
        try
            frame = robot.capture();
            filename = ['ElegooCam_Snapshot_', datestr(now, 'yyyymmdd_HHMMSS'), '.png'];
            imwrite(frame, filename);
            uialert(fig, ['Snapshot successfully captured and saved as ', filename], 'Snapshot Saved');
        catch ME
            uialert(fig, ['Failed to grab camera frame: ', ME.message], 'Camera Error');
        end
    end

    function saveWorkspaceData()
        carData = struct(...
            'time', timeBuffer, ...
            'gyro', gyroBuffer, ...
            'accelY', accelYBuffer, ...
            'range', rangeBuffer, ...
            'line', lineBuffer, ...
            'timestamp', datetime('now') ...
        );
        assignin('base', 'carDriveData', carData);
        uialert(fig, 'Sensor and drive arrays successfully saved to workspace variable "carDriveData".', 'Workspace Saved');
    end

    function takeUISnapshot(targetFig)
        img = getframe(targetFig);
        filename = ['ElegooUI_Snapshot_', datestr(now, 'yyyymmdd_HHMMSS'), '.png'];
        imwrite(img.cdata, filename);
        uialert(fig, ['UI snapshot successfully exported to ', filename], 'Snapshot Exported');
    end
end

function safeClose(robot)
    try
        robot.stop();
        robot.close();
    catch
    end
end

function closeUI(fig, timerObj, robot)
    try
        stop(timerObj);
        delete(timerObj);
        safeClose(robot);
    catch
    end
    delete(fig);
end
