% live_cir_stream.m
% Real-time CIR stream visualizer for DW3000 RX output using FRAME_BEGIN/FRAME_END markers.
% Update `serialPort` below to match your Arduino port (e.g., "COM7" or "/dev/ttyACM0").

serialPort = "COM7";  % CHANGE THIS FOR YOUR SETUP
baudRate   = 115200;   % Must match Arduino Serial.begin

% List available ports to guide selection.
availablePorts = serialportlist("available");
if isempty(availablePorts)
    error("No available serial ports detected. Plug in the Arduino and try again.");
end

fprintf("Available ports:\n");
disp(availablePorts.');

% Validate configured port.
if ~any(strcmp(serialPort, availablePorts))
    error("Configured port %s not found. Available ports: %s", serialPort, strjoin(availablePorts, ", "));
end

% Clean up any lingering connection on the same port.
legacyObj = instrfind("Port", serialPort); %#ok<CLIFIND>
if ~isempty(legacyObj)
    fclose(legacyObj);
    delete(legacyObj);
end

% Open the serial port using the modern serialport API.
try
    s = serialport(serialPort, baudRate, "Timeout", 1);
catch connErr
    error("Failed to open %s @ %d baud: %s", serialPort, baudRate, connErr.message);
end
configureTerminator(s, "LF");
flush(s);

cleanupObj = onCleanup(@() closeSerial(s)); %#ok<NASGU>
fprintf("Listening on %s @ %d baud... Press Ctrl+C to stop.\n", serialPort, baudRate);

% Pre-create the figure and line handles for smooth updates.
figure(1); clf;
subplot(3,1,1); hReal = plot(nan, nan, '-o'); grid on; xlabel('Sample Index'); ylabel('Real (I)'); title('CIR Real Part');
subplot(3,1,2); hImag = plot(nan, nan, '-o'); grid on; xlabel('Sample Index'); ylabel('Imag (Q)'); title('CIR Imag Part');
subplot(3,1,3); hMag  = plot(nan, nan, '-o'); grid on; xlabel('Sample Index'); ylabel('Magnitude'); title('CIR Magnitude');

buffering = false;
idxVec = [];
realVec = [];
imagVec = [];
magVec  = [];

while true
    try
        rawLine = readline(s);
    catch readErr
        warning("Serial read error: %s", readErr.message);
        pause(0.05);
        continue;
    end

    line = strtrim(rawLine);
    if isempty(line)
        continue;
    end

    if strcmp(line, "FRAME_BEGIN")
        buffering = true;
        idxVec = [];
        realVec = [];
        imagVec = [];
        magVec  = [];
        continue;
    elseif strcmp(line, "FRAME_END")
        if buffering && ~isempty(idxVec)
            set(hReal, 'XData', idxVec, 'YData', realVec);
            set(hImag, 'XData', idxVec, 'YData', imagVec);
            set(hMag,  'XData', idxVec, 'YData', magVec);
            drawnow limitrate;
        end
        buffering = false;
        continue;
    end

    if buffering
        tokens = split(line, ',');
        if numel(tokens) ~= 4
            continue; % malformed line, ignore
        end

        vals = str2double(tokens);
        if any(isnan(vals))
            continue; % malformed numbers
        end

        idxVec(end+1)  = vals(1); %#ok<AGROW>
        realVec(end+1) = vals(2); %#ok<AGROW>
        imagVec(end+1) = vals(3); %#ok<AGROW>
        magVec(end+1)  = vals(4); %#ok<AGROW>
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
end

