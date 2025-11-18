% cir_stream_plot.m
% Real-time streaming and plotting of DW3000 CIR samples coming from the
% Arduino RX sketch. The RX sketch must print:
%   FRAME_BEGIN
%   idx,real,imag,mag
%   ... (repeated for each sample)
%   FRAME_END
% This script buffers the lines between FRAME_BEGIN and FRAME_END, parses
% them, and updates three plots (real, imaginary, magnitude) for every
% received frame.

% -------------------------------------------------------------------------
% User configuration
% -------------------------------------------------------------------------
serialPort = "COM3";   % Change to your port, e.g., "COM6" or "/dev/ttyACM0"
baudRate   = 115200;   % Must match Serial.begin in the Arduino sketch

availablePorts = serialportlist("available");
if isempty(availablePorts)
    error("No available serial ports detected. Plug in the Arduino and retry.");
end
if ~any(strcmp(serialPort, availablePorts))
    error("Configured port %s not found. Available: %s", serialPort, strjoin(availablePorts.', ", "));
end

% Clean up any lingering serial object on the same port
cleanupSerial(serialPort);

% Open the serial port
sp = serialport(serialPort, baudRate, "Timeout", 1);
configureTerminator(sp, "LF");
flush(sp);
cleanupObj = onCleanup(@() cleanupSerial(serialPort, sp)); %#ok<NASGU>

% Prepare figure and plots
fig = figure('Name', 'DW3000 CIR Streaming');
t = tiledlayout(fig, 3, 1, "TileSpacing", "compact");
nexttile(t, 1);
realPlot = plot(nan, nan, '-b');
ylabel('Real (I)'); grid on;
nexttile(t, 2);
imagPlot = plot(nan, nan, '-r');
ylabel('Imag (Q)'); grid on;
nexttile(t, 3);
magPlot = plot(nan, nan, '-k');
ylabel('Magnitude'); xlabel('Sample index'); grid on;
title(t, 'DW3000 CIR Streaming');

disp("Listening for CIR frames... Press Ctrl+C or close the figure to stop.");

buffering = false;
frameLines = string.empty(0, 1);

while ishandle(fig)
    try
        raw = readline(sp);
        if isempty(raw)
            pause(0.05);
            continue;
        end
        line = strtrim(string(raw));
    catch readErr
        warning("Serial read error: %s", readErr.message);
        pause(0.05);
        continue;
    end

    if line == "FRAME_BEGIN"
        buffering = true;
        frameLines = string.empty(0, 1);
        continue;
    elseif line == "FRAME_END"
        if buffering && ~isempty(frameLines)
            [idx, realPart, imagPart, mag] = parseFrame(frameLines);
            if ~isempty(idx)
                set(realPlot, 'XData', idx, 'YData', realPart);
                set(imagPlot, 'XData', idx, 'YData', imagPart);
                set(magPlot,  'XData', idx, 'YData', mag);
                drawnow limitrate;
            end
        end
        buffering = false;
        frameLines = string.empty(0, 1);
        continue;
    end

    if buffering
        frameLines(end+1) = line; %#ok<AGROW>
    end
end

disp("Stopped CIR streaming.");

% -------------------------------------------------------------------------
% Helper functions
% -------------------------------------------------------------------------
function [idx, realPart, imagPart, mag] = parseFrame(lines)
% Parse lines of the form "idx,real,imag,mag" into numeric arrays.
    idx = [];
    realPart = [];
    imagPart = [];
    mag = [];
    for k = 1:numel(lines)
        tokens = split(lines(k), ',');
        if numel(tokens) ~= 4
            continue; % skip malformed lines
        end
        vals = str2double(tokens);
        if any(isnan(vals))
            continue; % skip malformed numbers
        end
        idx(end+1) = vals(1); %#ok<AGROW>
        realPart(end+1) = vals(2); %#ok<AGROW>
        imagPart(end+1) = vals(3); %#ok<AGROW>
        mag(end+1) = vals(4); %#ok<AGROW>
    end
end

function cleanupSerial(portName, spObj)
% Close and delete any serialport or legacy serial object on the port.
    % serialport objects (modern API)
    if nargin > 1 && ~isempty(spObj) && isvalid(spObj) && strcmpi(spObj.Port, portName)
        try
            flush(spObj);
        catch
        end
        try
            delete(spObj);
        catch
        end
    end
    existing = serialportfind("Port", portName);
    for k = 1:numel(existing)
        try
            delete(existing(k));
        catch
        end
    end
    % Legacy serial objects
    legacyObj = instrfind("Port", portName); %#ok<CLIFIND>
    if ~isempty(legacyObj)
        fclose(legacyObj);
        delete(legacyObj);
    end
end
