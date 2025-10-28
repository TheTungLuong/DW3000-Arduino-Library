function cir_stream_plot(port, baudrate, totalSamples)
% CIR_STREAM_PLOT  Stream CIR samples from the DW3000 RX example via serial.
%   CIR_STREAM_PLOT(PORT) opens the serial PORT at 115200 baud and plots the
%   magnitude^2 of the CIR samples in real time as they are emitted by the
%   dw3000\_rx\_cir\_stream Arduino sketch.
%
%   CIR_STREAM_PLOT(PORT, BAUDRATE, TOTALSAMPLES) allows overriding the
%   serial configuration as well as the number of samples expected per
%   frame. TOTALSAMPLES must match CIR_TOTAL_SAMPLES in the Arduino sketch.
%
%   Example:
%       cir_stream_plot("COM4", 115200, 256);
%
%   Press Ctrl+C in the MATLAB command window to stop streaming.

if nargin < 2 || isempty(baudrate)
    baudrate = 115200;
end
if nargin < 3 || isempty(totalSamples)
    totalSamples = 256;
end

s = serialport(port, baudrate, "Timeout", 2);
configureTerminator(s, "LF");
flush(s);

cleanupObj = onCleanup(@()closeSerial(s));

figureHandle = figure('Name', 'DW3000 CIR Stream', 'NumberTitle', 'off');
axesHandle = axes('Parent', figureHandle);
plotHandle = plot(axesHandle, zeros(totalSamples, 1));
xlabel(axesHandle, 'Sample Index');
ylabel(axesHandle, '|CIR|^2');
frameLabel = title(axesHandle, 'Waiting for data...');

realBuffer = zeros(totalSamples, 1);
imagBuffer = zeros(totalSamples, 1);
magBuffer = zeros(totalSamples, 1);
complexMask = false(totalSamples, 1);
dataBuffer = zeros(totalSamples, 1);
frameIndex = -1;
sampleOffset = NaN;

while isvalid(figureHandle)
    try
        line = strtrim(readline(s));
    catch readErr
        warning('CIR stream read error: %s', readErr.message);
        pause(0.1);
        continue;
    end

    if isempty(line)
        continue;
    end

    if startsWith(line, "CIR_BEGIN", 'IgnoreCase', true)
        tokens = split(line, ',');
        if numel(tokens) >= 2
            frameIndex = str2double(tokens{2});
        end
        realBuffer(:) = 0;
        imagBuffer(:) = 0;
        magBuffer(:) = 0;
        complexMask(:) = false;
        dataBuffer(:) = 0;
        sampleOffset = NaN;
    elseif startsWith(line, "CIR_END", 'IgnoreCase', true)
        dataBuffer = realBuffer.^2 + imagBuffer.^2;
        missingMagnitude = ~complexMask & (magBuffer ~= 0);
        dataBuffer(missingMagnitude) = magBuffer(missingMagnitude);
        set(plotHandle, 'YData', dataBuffer);
        if ~isnan(frameIndex)
            set(frameLabel, 'String', sprintf('DW3000 CIR frame %d', frameIndex));
        else
            set(frameLabel, 'String', 'DW3000 CIR frame (unknown)');
        end
        drawnow limitrate;
    elseif startsWith(line, "CIR,", 'IgnoreCase', true)
        tokens = split(line, ',');
        if numel(tokens) < 6
            continue;
        end
        sampleIndex = str2double(tokens{3});
        if isnan(sampleOffset)
            sampleOffset = sampleIndex;
        end
        bufferIndex = round(sampleIndex - sampleOffset + 1);
        if bufferIndex < 1 || bufferIndex > totalSamples
            continue;
        end
        realVal = str2double(tokens{4});
        imagVal = str2double(tokens{5});
        if ~isnan(realVal)
            realBuffer(bufferIndex) = realVal;
        end
        if ~isnan(imagVal)
            imagBuffer(bufferIndex) = imagVal;
        end
        complexMask(bufferIndex) = (~isnan(realVal)) && (~isnan(imagVal));
        magVal = str2double(tokens{6});
        if ~isnan(magVal)
            magBuffer(bufferIndex) = magVal;
        end
    else
        fprintf('%s\n', line);
    end
end

end

function closeSerial(s)
if isempty(s)
    return;
end
try
    if isvalid(s)
        flush(s);
    end
catch
end
end
