% live_cir_stream.m
% Real-time CIR stream visualizer for DW3000 RX output.
% Replace SERIAL_PORT with your Arduino's port (e.g., "COM4" on Windows or "/dev/ttyUSB0" on Linux).

function live_cir_stream()
    serialPort = "COMx";   % TODO: set to your port
    baudRate   = 115200;    % Must match Arduino Serial.begin

    s = serialport(serialPort, baudRate, "Timeout", 1);
    configureTerminator(s, "LF");
    flush(s);
    cleanupObj = onCleanup(@() cleanupSerial(s));

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

function cleanupSerial(s)
    if ~isempty(s) && isvalid(s)
        flush(s);
        clear s;
    end
end
