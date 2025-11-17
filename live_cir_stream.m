% live_cir_stream.m
% Real-time CIR stream visualizer for DW3000 RX output.
% -------------------------------------------------------------------------
% Set your COM port and baud rate below. Example: "COM6" on Windows or
% "/dev/ttyUSB0" on Linux. If connection fails, confirm the Arduino is
% plugged in, the port matches Device Manager / lsusb output, and that the
% port is not open elsewhere (e.g., Arduino IDE Serial Monitor).
% -------------------------------------------------------------------------

function live_cir_stream()
    % >>> CONFIGURE THESE FOR YOUR SETUP <<<
    serialPort = "COM6";   % CHANGE THIS IF NEEDED
    baudRate   = 115200;    % Must match Arduino Serial.begin

    % Show available ports to help pick the right one
    availablePorts = serialportlist("available");
    if isempty(availablePorts)
        fprintf(2, "No available serial ports detected. Plug in the Arduino and try again.\n");
    else
        fprintf("Available ports:\n");
        disp(availablePorts.');
    end

    % Validate chosen port exists before attempting to open
    if ~any(strcmp(serialPort, availablePorts))
        error("Configured port %s not found. Update 'serialPort' to one of the available ports.", serialPort);
    end

    % Close/clear any lingering serial object on the same port
    if exist('s', 'var') && isa(s, 'serialport') && isvalid(s) && strcmpi(s.Port, serialPort)
        closeSerial(s);
        clear s;
    end
    legacyObj = instrfind("Port", serialPort); %#ok<CLIFIND>
    if ~isempty(legacyObj)
        fclose(legacyObj);
        delete(legacyObj);
    end

    % Attempt to open the serial port safely
    try
        s = serialport(serialPort, baudRate, "Timeout", 1);
    catch connErr
        fprintf(2, "Failed to open %s @ %d baud: %s\n", serialPort, baudRate, connErr.message);
        fprintf(2, "Check if the Arduino is plugged in, the correct COM port is used, and that the port is not already open in Arduino IDE or another program.\n");
        return;
    end

    configureTerminator(s, "LF");
    flush(s);
    cleanupObj = onCleanup(@() closeSerial(s)); %#ok<NASGU>

    currentFrame = -1;
    sampleIdx = [];
    realPart = [];
    imagPart = [];
    magnitude = [];

    fprintf("Listening on %s @ %d baud...\n", serialPort, baudRate);
    fprintf("Press Ctrl+C to stop.\n");

    while true
        try
            line = strtrim(readline(s));
        catch readErr
            warning("Serial read error: %s", readErr.message);
            pause(0.05);
            continue;
        end

        if startsWith(line, "CIR,", "IgnoreCase", true)
            tokens = split(line, ',');
            if numel(tokens) ~= 6
                continue; % malformed line
            end
            frameId = str2double(tokens{2});
            idxVal  = str2double(tokens{3});
            realVal = str2double(tokens{4});
            imagVal = str2double(tokens{5});
            magVal  = str2double(tokens{6});

            if any(isnan([frameId, idxVal, realVal, imagVal, magVal]))
                continue; % skip malformed numbers
            end

            if currentFrame ~= frameId
                % New frame detected, reset buffers
                currentFrame = frameId;
                sampleIdx = [];
                realPart = [];
                imagPart = [];
                magnitude = [];
            end

            sampleIdx(end+1) = idxVal; %#ok<AGROW>
            realPart(end+1) = realVal; %#ok<AGROW>
            imagPart(end+1) = imagVal; %#ok<AGROW>
            magnitude(end+1) = magVal; %#ok<AGROW>

        elseif startsWith(line, "CIR_END", "IgnoreCase", true)
            tokens = split(line, ',');
            if numel(tokens) < 2
                continue;
            end
            endFrame = str2double(tokens{2});
            if isnan(endFrame) || endFrame ~= currentFrame
                continue; % mismatch, ignore
            end

            % Plot the full frame
            figure(1); clf;
            subplot(3,1,1);
            plot(sampleIdx, realPart, '-o'); grid on;
            xlabel('Sample Index'); ylabel('Real (I)');
            title(sprintf('Frame %d - Real Part', currentFrame));

            subplot(3,1,2);
            plot(sampleIdx, imagPart, '-o'); grid on;
            xlabel('Sample Index'); ylabel('Imag (Q)');
            title(sprintf('Frame %d - Imaginary Part', currentFrame));

            subplot(3,1,3);
            plot(sampleIdx, magnitude, '-o'); grid on;
            xlabel('Sample Index'); ylabel('Magnitude');
            title(sprintf('Frame %d - Magnitude', currentFrame));

            drawnow;
        else
            % Ignore unrelated lines
        end
    end
end

function closeSerial(s)
    if isempty(s) || ~isvalid(s)
        return;
    end
    try
        flush(s);
    catch
    end
    try
        delete(s);
    catch
    end
    try
        clear s;
    catch
    end
end
