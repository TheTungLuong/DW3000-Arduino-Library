function capture = stream_dw3000_cir(serialPort, numSamples)
%STREAM_DW3000_CIR Capture and plot DW3000 CIR samples over Serial.
%   DATA = STREAM_DW3000_CIR(SERIALPORT, NUMSAMPLES) opens SERIALPORT (for
%   example "COM4" on Windows or "/dev/ttyACM0" on Linux/macOS) at
%   921600 baud, reads NUMSAMPLES CSV rows produced by the CIR_Streamer
%   Arduino sketch, and plots the real, imaginary, and magnitude traces
%   using standard MATLAB line plots. The returned DATA struct contains the
%   numeric vectors that were plotted so additional analysis can be done.
%
%   SERIALPORT is optional. If omitted or empty, an interactive prompt is
%   shown listing the currently available serial devices so you can select
%   the Arduino's port without editing this file.
%
%   NUMSAMPLES defaults to 1016 (the CIR_Streamer sketch's preamble
%   capture length) when not provided.
%
%   Before running this helper:
%     * Upload the CIR_Streamer.ino sketch to your Arduino board.
%     * Ensure the board is connected; the sketch automatically streams a
%       capture on reset and whenever it receives the character 'c'.
%
%   Example:
%     % Prompt for a port and capture the default 1016 preamble samples
%     data = stream_dw3000_cir();
%
%     % Capture 512 samples from an explicitly provided port
%     data = stream_dw3000_cir("COM5", 512);
%
%   The script closes the serial connection automatically once the
%   requested number of rows has been read.

arguments
    serialPort (1, :) char = ''
    numSamples (1, 1) {mustBePositive, mustBeInteger} = 1016
end

serialPort = selectSerialPort(serialPort);

baudRate = 921600;
sp = serialport(serialPort, baudRate, "Timeout", 10);
configureTerminator(sp, "LF");
cleanup = onCleanup(@cleanupSerial);

flush(sp);
pause(2.0);  % Allow the Arduino time to reset and begin streaming.

maxHeaderReads = 50;
header = "";
headerInfo = struct('frameIdx', 0, 'indexIdx', 0, 'realIdx', 0, ...
    'imagIdx', 0, 'magIdx', 0);
while maxHeaderReads > 0
    rawLine = safeReadline(sp);
    if strlength(rawLine) == 0
        maxHeaderReads = maxHeaderReads - 1;
        continue;
    end

    candidate = strtrim(rawLine);
    if startsWith(candidate, "#")
        parsed = parseHeader(candidate);
        if parsed.realIdx ~= 0 && parsed.imagIdx ~= 0
            header = candidate;
            headerInfo = parsed;
            break;
        end
    end

    maxHeaderReads = maxHeaderReads - 1;
end

if header == ""
    warning("Did not receive a recognizable header. Assuming index,I,Q,mag order.");
    headerInfo.indexIdx = 1;
    headerInfo.realIdx = 2;
    headerInfo.imagIdx = 3;
    headerInfo.magIdx = 4;
end

samplesRead = 0;
indices = zeros(numSamples, 1);
realVals = zeros(numSamples, 1);
imagVals = zeros(numSamples, 1);
magVals = zeros(numSamples, 1);
frameVals = NaN(numSamples, 1);
captureAttributes = struct();

stallCounter = 0;

while samplesRead < numSamples
    rawLine = safeReadline(sp);
    if strlength(rawLine) == 0
        stallCounter = stallCounter + 1;
        if stallCounter > max(100, numSamples * 2)
            error('Timed out waiting for CIR samples. Read %d of %d rows.', samplesRead, numSamples);
        end
        continue;
    end

    trimmed = strtrim(rawLine);

    if strlength(trimmed) == 0
        continue;
    end

    if startsWith(trimmed, "#")
        if startsWith(trimmed, "#INFO", 'IgnoreCase', true)
            attrs = parseInfoAttributes(trimmed);
            if ~isempty(fieldnames(attrs))
                captureAttributes = mergeAttributes(captureAttributes, attrs);
                displayAttributes(captureAttributes);
            end
            continue;
        end
        parsed = parseHeader(trimmed);
        if parsed.realIdx ~= 0 && parsed.imagIdx ~= 0
            headerInfo = parsed;
        end
        continue;
    end

    tokens = split(trimmed, ',');
    values = str2double(tokens);

    requiredCount = max([headerInfo.frameIdx, headerInfo.indexIdx, ...
        headerInfo.realIdx, headerInfo.imagIdx, headerInfo.magIdx, 1]);
    if numel(values) < requiredCount
        warning("Skipping malformed row: %s", trimmed);
        continue;
    end

    realVal = values(headerInfo.realIdx);
    imagVal = values(headerInfo.imagIdx);

    if any(isnan([realVal, imagVal]))
        warning("Skipping row containing NaN values: %s", trimmed);
        continue;
    end

    idxVal = samplesRead + 1;
    if headerInfo.indexIdx ~= 0 && headerInfo.indexIdx <= numel(values)
        idxVal = values(headerInfo.indexIdx);
    end

    magVal = NaN;
    if headerInfo.magIdx ~= 0 && headerInfo.magIdx <= numel(values)
        magVal = values(headerInfo.magIdx);
    end
    if isnan(magVal)
        magVal = sqrt(realVal.^2 + imagVal.^2);
    end

    frameVal = NaN;
    if headerInfo.frameIdx ~= 0 && headerInfo.frameIdx <= numel(values)
        frameVal = values(headerInfo.frameIdx);
    end

    samplesRead = samplesRead + 1;
    indices(samplesRead) = idxVal;
    realVals(samplesRead) = realVal;
    imagVals(samplesRead) = imagVal;
    magVals(samplesRead) = magVal;
    frameVals(samplesRead) = frameVal;
    stallCounter = 0;

end

% Trim arrays in case malformed rows were skipped
indices = indices(1:samplesRead);
realVals = realVals(1:samplesRead);
imagVals = imagVals(1:samplesRead);
magVals = magVals(1:samplesRead);
frameVals = frameVals(1:samplesRead);

figureHandle = figure('Name', 'DW3000 CIR Capture', 'NumberTitle', 'off');
set(figureHandle, 'Color', 'w');

tiledlayout(3, 1, 'TileSpacing', 'compact');

axReal = nexttile;
plot(axReal, indices, realVals, 'Color', [0.0, 0.45, 0.74]);
title(axReal, 'CIR Real Component');
xlabel(axReal, 'Sample Index');
ylabel(axReal, 'Amplitude');
grid(axReal, 'on');

axImag = nexttile;
plot(axImag, indices, imagVals, 'Color', [0.85, 0.33, 0.10]);
title(axImag, 'CIR Imaginary Component');
xlabel(axImag, 'Sample Index');
ylabel(axImag, 'Amplitude');
grid(axImag, 'on');

axMag = nexttile;
plot(axMag, indices, magVals, 'Color', [0.47, 0.67, 0.19]);
title(axMag, 'CIR Magnitude');
xlabel(axMag, 'Sample Index');
ylabel(axMag, 'Magnitude');
grid(axMag, 'on');

fprintf('Captured %d samples from %s at %d baud.\n', samplesRead, serialPort, baudRate);

capture = struct('index', indices, 'real', realVals, ...
    'imag', imagVals, 'magnitude', magVals);

if any(~isnan(frameVals))
    capture.frame = frameVals;
end

if ~isempty(fieldnames(captureAttributes))
    capture.attributes = captureAttributes;
end

clear sp;

    function chosenPort = selectSerialPort(initialPort)
        if ~(isstring(initialPort) || ischar(initialPort))
            error('Serial port must be specified as text.');
        end

        if ~isempty(strtrim(initialPort))
            chosenPort = char(initialPort);
            return;
        end

        availablePorts = serialportlist("available");
        if isempty(availablePorts)
            error(['No available serial ports detected. Connect the Arduino and ' ...
                'specify its port explicitly when calling stream_dw3000_cir.']);
        end

        fprintf('Available serial ports:\n');
        for ii = 1:numel(availablePorts)
            fprintf('  %d) %s\n', ii, char(availablePorts(ii)));
        end

        selectionPrompt = sprintf('Select port [1-%d]: ', numel(availablePorts));
        selection = input(selectionPrompt);

        if isempty(selection) || ~isnumeric(selection) || ~isscalar(selection)
            error('Invalid selection: please enter the number corresponding to the desired port.');
        end

        selection = floor(selection);
        if selection < 1 || selection > numel(availablePorts)
            error('Selection %d is out of range.', selection);
        end

        chosenPort = char(availablePorts(selection));
    end

    function line = safeReadline(port)
        line = "";
        try
            line = readline(port);
        catch me
            if ~contains(me.message, "Timeout")
                rethrow(me);
            end
        end
    end

    function cleanupSerial()
        if exist('sp', 'var') && ~isempty(sp)
            try
                flush(sp);
            catch
                % Ignore flush errors during cleanup
            end
        end
    end

    function attrs = parseInfoAttributes(line)
        attrs = struct();

        if strlength(line) == 0
            return;
        end

        if startsWith(line, "#")
            line = extractAfter(line, 1);
        end

        if strlength(line) == 0
            return;
        end

        tokens = split(line, ',');
        tokens = strip(tokens);

        if isempty(tokens)
            return;
        end

        firstToken = tokens(1);
        if strlength(firstToken) > 0
            topicName = lower(strrep(strjoin(split(firstToken)), ' ', '_'));
            attrs.topic = topicName;
        end

        for ii = 2:numel(tokens)
            kv = split(tokens(ii), '=');
            if numel(kv) ~= 2
                continue;
            end

            key = strtrim(lower(kv(1)));
            if strlength(key) == 0
                continue;
            end
            key = matlab.lang.makeValidName(key);

            valueStr = strtrim(kv(2));
            valueNum = str2double(valueStr);
            if ~isnan(valueNum)
                attrs.(key) = valueNum;
            else
                attrs.(key) = valueStr;
            end
        end
    end

    function merged = mergeAttributes(existing, incoming)
        merged = existing;
        if isempty(fieldnames(incoming))
            return;
        end

        keys = fieldnames(incoming);
        for ii = 1:numel(keys)
            merged.(keys{ii}) = incoming.(keys{ii});
        end
    end

    function displayAttributes(attrs)
        if isempty(fieldnames(attrs))
            return;
        end

        topic = '';
        if isfield(attrs, 'topic')
            topic = attrs.topic;
        end

        otherFields = setdiff(fieldnames(attrs), {'topic'});

        if strlength(topic) > 0
            readableTopic = strrep(topic, '_', ' ');
            fprintf('Capture %s', readableTopic);
        else
            fprintf('Capture attributes');
        end

        if isempty(otherFields)
            fprintf('.\n');
            return;
        end

        fprintf(': ');
        for idx = 1:numel(otherFields)
            key = otherFields{idx};
            value = attrs.(key);
            if isnumeric(value)
                valueStr = num2str(value);
            else
                valueStr = char(value);
            end
            fprintf('%s=%s', key, valueStr);
            if idx < numel(otherFields)
                fprintf(', ');
            end
        end
        fprintf('\n');
    end

    function info = parseHeader(line)
        info = struct('frameIdx', 0, 'indexIdx', 0, 'realIdx', 0, ...
            'imagIdx', 0, 'magIdx', 0);

        if strlength(line) == 0
            return;
        end

        if startsWith(line, "#")
            line = extractAfter(line, 1);
        end

        if strlength(line) == 0
            return;
        end

        tokens = split(line, ',');
        tokens = strip(lower(tokens));

        for ii = 1:numel(tokens)
            token = tokens(ii);
            switch token
                case "frame"
                    info.frameIdx = ii;
                case "index"
                    info.indexIdx = ii;
                case {"i", "real"}
                    info.realIdx = ii;
                case {"q", "imag"}
                    info.imagIdx = ii;
                case {"mag", "magnitude"}
                    info.magIdx = ii;
            end
        end

        if info.realIdx == 0 || info.imagIdx == 0
            info = struct('frameIdx', 0, 'indexIdx', 0, 'realIdx', 0, ...
                'imagIdx', 0, 'magIdx', 0);
        end
    end

end
