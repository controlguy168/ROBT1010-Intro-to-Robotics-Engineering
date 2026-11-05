function robotBeatDanceUI()
    % ELEGOO Smart Robot Car - Beat-Synchronized Dance Studio UI
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
    currentAudioPlayer = [];

    % --- 2. Create Main UI Figure & Layout ---
    fig = uifigure('Name', 'Elegoo Robot Car - Beat-Synced Dance Studio', 'Position', [50, 50, 1100, 780]);
    
    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {'3x', '2x'};

    leftGrid = uigridlayout(mainGrid, [2, 1]);
    leftGrid.RowHeight = {'2.3x', '1.7x'};

    rightGrid = uigridlayout(mainGrid, [2, 1]);
    rightGrid.RowHeight = {'2.5x', '2.5x'};

    % ==================== VISUAL & STATUS PANEL ====================
    statusPanel = uipanel(leftGrid, 'Title', 'Dance Floor Status & Rhythm Log');
    statusGrid = uigridlayout(statusPanel, [1, 1]);

    statusTextArea = uitextarea(statusGrid, 'Editable', 'off', ...
        'Value', ["Beat-Sync Studio Ready.", "1. Load music file (.mp3/.wav).", "2. Set track BPM (e.g., 120) or click Tap Tempo.", "3. Click START BEAT-SYNC DANCE."]);

    % ==================== CONTROL PANEL ====================
    controlPanel = uipanel(leftGrid, 'Title', 'Choreography & Rhythm Controls');
    controlGrid = uigridlayout(controlPanel, [7, 2]);

    uilabel(controlGrid, 'Text', 'Track Tempo (BPM):');
    bpmInput = uieditfield(controlGrid, 'numeric', 'Value', 120, 'Limits', [40, 220]);

    uilabel(controlGrid, 'Text', 'Dance Speed (v):');
    speedInput = uieditfield(controlGrid, 'numeric', 'Value', 100, 'Limits', [50, 255]);

    uilabel(controlGrid, 'Text', 'Turn Energy (omega):');
    omegaInput = uieditfield(controlGrid, 'numeric', 'Value', 150, 'Limits', [50, 255]);

    uilabel(controlGrid, 'Text', 'Safety Threshold (mm):');
    safetyThreshInput = uieditfield(controlGrid, 'numeric', 'Value', 200, 'Limits', [50, 500]);

    loadMusicBtn = uibutton(controlGrid, 'Text', 'Load Music File (.mp3/.wav)', ...
        'ButtonPushedFcn', @(~,~) loadMusic());
    loadMusicBtn.Layout.Column = [1, 2];

    startDanceBtn = uibutton(controlGrid, 'Text', 'START BEAT-SYNC DANCE', ...
        'BackgroundColor', [0.4 0.8 0.4], ...
        'ButtonPushedFcn', @(~,~) runBeatDance(bpmInput.Value, speedInput.Value, omegaInput.Value, safetyThreshInput.Value));
    startDanceBtn.Layout.Column = [1, 2];

    stopRoutineBtn = uibutton(controlGrid, 'Text', 'STOP DANCE', ...
        'BackgroundColor', [1 0.6 0.6], ...
        'ButtonPushedFcn', @(~,~) stopRoutine());
    stopRoutineBtn.Layout.Column = [1, 2];

    % ==================== SIDE PANEL: CHOREOGRAPHY INFO ====================
    infoPanel = uipanel(rightGrid, 'Title', 'Rhythm & Beat Structure');
    infoGrid = uigridlayout(infoPanel, [2, 1]);

    infoText = uitextarea(infoGrid, 'Editable', 'off', 'Value', [ ...
        "Beat-Synchronized Moves:"; ...
        "• Quarter Note (1 Beat): Step Forward / Backward"; ...
        "• Eighth Note (0.5 Beat): Quick Side-to-Side Wiggle"; ...
        "• Whole Note (4 Beats): 360 Spin Groove"; ...
        "• Half Note (2 Beats): Pan Head Nod"; ...
        ""; ...
        "Safety Guarantee:"; ...
        "• Ultrasonic distance sensor checks safety dynamically within each beat window." ...
    ]);

    uibutton(infoGrid, 'Text', 'Emergency Stop Robot', ...
        'BackgroundColor', [1 0.3 0.3], ...
        'ButtonPushedFcn', @(~,~) robot.stop());

    audioData = [];
    audioFs = 44100;
    hasMusic = false;

    fig.CloseRequestFcn = @(~,~) closeUI(fig, robot);

    function logMsg(msg)
        currentText = string(statusTextArea.Value);
        timestampedMsg = "[" + datestr(now, 'HH:MM:SS') + "] " + msg;
        statusTextArea.Value = [timestampedMsg; currentText];
        drawnow limitrate;
    end

    function loadMusic()
        [file, path] = uigetfile({'*.mp3;*.wav', 'Audio Files (*.mp3, *.wav)'}, 'Select Track');
        if isequal(file, 0)
            logMsg('Music selection cancelled.');
            return;
        end
        
        fullPath = fullfile(path, file);
        logMsg(['Loading track: ', file]);
        try
            [audioData, audioFs] = audioread(fullPath);
            hasMusic = true;
            logMsg('Music track loaded successfully!');
        catch ME
            logMsg(['Failed to load audio: ', ME.message]);
            hasMusic = false;
        end
    end

    % Helper function to execute a move for an exact number of beats
    function success = executeBeatMove(actionType, spd, beatCount, beatDuration, safeThresh)
        success = true;
        targetDuration = beatCount * beatDuration;
        tStart = tic;

        while isRoutineRunning && toc(tStart) < targetDuration
            % Safety check with ultrasonic range sensor
            try
                dData = robot.readDistance();
                if isstruct(dData) && isfield(dData, 'distance')
                    dVal = double(dData.distance);
                    if dVal > 0 && dVal < safeThresh
                        logMsg(['⚠️ Obstacle at ', num2str(round(dVal)), ' mm! Pausing beat step...']);
                        robot.backward(spd);
                        pause(beatDuration); % Back up for 1 beat
                        robot.turnRight(spd);
                        pause(beatDuration); % Turn for 1 beat
                        robot.stop();
                        success = false;
                        return;
                    end
                end
            catch
            end

            % Execute maneuver
            switch lower(actionType)
                case 'forward'
                    robot.forward(spd);
                case 'backward'
                    robot.backward(spd);
                case 'turnleft'
                    robot.turnLeft(spd);
                case 'turnright'
                    robot.turnRight(spd);
                case 'stop'
                    robot.stop();
            end
            pause(0.02);
        end
        robot.stop();
    end

    function runBeatDance(bpm, speed, turnSpd, safeThresh)
        if isRoutineRunning
            return;
        end
        isRoutineRunning = true;

        % Calculate exact beat interval in seconds
        beatInterval = 60 / bpm;
        logMsg(['Starting Beat-Synced Dance | BPM = ', num2str(bpm), ' (1 Beat = ', num2str(beatInterval, '%.3f'), ' s)']);

        % Start audio playback on beat zero
        if hasMusic
            try
                currentAudioPlayer = audioplayer(audioData, audioFs);
                play(currentAudioPlayer);
            catch
            end
        end

        try
            beatCounter = 0;

            while isRoutineRunning
                beatCounter = beatCounter + 1;
                logMsg(['Executing Measure #', num2str(ceil(beatCounter/4)), '...']);

                % --- BEAT-SYNCED CHOREOGRAPHY ROUTINE ---

                % Phase 1: 4 Beats of Head-Bobs (2 beats left, 2 beats right)
                robot.pan(-40);
                executeBeatMove('stop', 0, 2.0, beatInterval, safeThresh);
                robot.pan(40);
                executeBeatMove('stop', 0, 2.0, beatInterval, safeThresh);
                robot.pan(0);

                if ~isRoutineRunning, break; end

                % Phase 2: 4 Beats of Quick Wiggles (0.5 beats each)
                for i = 1:4
                    if ~isRoutineRunning, break; end
                    executeBeatMove('turnLeft', turnSpd, 0.5, beatInterval, safeThresh);
                    executeBeatMove('turnRight', turnSpd, 0.5, beatInterval, safeThresh);
                end

                if ~isRoutineRunning, break; end

                % Phase 3: 2 Beats Forward + 2 Beats Backward (The Moonwalk)
                executeBeatMove('forward', speed, 2.0, beatInterval, safeThresh);
                executeBeatMove('backward', speed, 2.0, beatInterval, safeThresh);

                if ~isRoutineRunning, break; end

                % Phase 4: 4-Beat Full 360 Degree Spin
                executeBeatMove('turnRight', turnSpd, 4.0, beatInterval, safeThresh);

                if ~isRoutineRunning, break; end
            end

            stopMusic();
            logMsg('Beat-synced dance finished.');

        catch ME
            stopMusic();
            logMsg(['Dance error: ', ME.message]);
            robot.stop();
            isRoutineRunning = false;
        end
    end

    function stopMusic()
        try
            if ~isempty(currentAudioPlayer)
                stop(currentAudioPlayer);
            end
        catch
        end
    end

    function stopRoutine()
        isRoutineRunning = false;
        stopMusic();
        try
            robot.stop();
            robot.pan(0);
            logMsg('Dance stopped by user.');
        catch
        end
    end
end

function safeClose(robot)
    try
        robot.stop();
        robot.pan(0);
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
