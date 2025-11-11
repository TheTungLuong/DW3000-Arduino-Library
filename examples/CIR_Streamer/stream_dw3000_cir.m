function captureTable = stream_dw3000_cir(serialPort, numSamples)
%STREAM_DW3000_CIR Capture and plot DW3000 CIR samples over Serial.
%   T = STREAM_DW3000_CIR(SERIALPORT, NUMSAMPLES) opens SERIALPORT (for
%   example "COM4" on Windows or "/dev/ttyACM0" on Linux/macOS) at
%   921600 baud, reads NUMSAMPLES CSV rows produced by the CIR_Streamer
%   Arduino sketch, streams the real, imaginary, and magnitude traces in
%   MATLAB, and returns a table containing the captured samples.
%
%   Before running this helper:
%     * Upload the CIR_Streamer.ino sketch to your Arduino board.
%     * Reset or power-cycle the board so the sketch outputs a single
%       capture beginning with the header line "#index,I,Q,mag".
%     * Update SERIALPORT to match the COM port used by your Arduino.
%
%   Example:
%     % Capture the default 1016 preamble samples on Windows COM5
%     capture = stream_dw3000_cir("COM5", 1016);
%
%   The script closes the serial connection automatically once the
%   requested number of rows has been read.

arguments
    serialPort (1, :) char
    numSamples (1, 1) {mustBePositive, mustBeInteger}
end

baudRate = 921600;
sp = serialport(serialPort, baudRate, "Timeout", 10);
configureTerminator(sp, "LF");
cleanup = onCleanup(@cleanupSerial);

flush(sp);

header = strtrim(readline(sp));
if ~strcmp(header, "#index,I,Q,mag")
    warning("Unexpected header received: %s", header);
end

figureHandle = figure('Name', 'DW3000 CIR Capture', 'NumberTitle', 'off');
set(figureHandle, 'Color', 'w');

tiledlayout(3, 1, 'TileSpacing', 'compact');
axReal = nexttile;
realLine = animatedline(axReal, 'Color', [0.0, 0.45, 0.74]);
title(axReal, 'CIR Real Component');
xlabel(axReal, 'Sample Index');
ylabel(axReal, 'Amplitude');

axImag = nexttile;
imagLine = animatedline(axImag, 'Color', [0.85, 0.33, 0.10]);
title(axImag, 'CIR Imaginary Component');
xlabel(axImag, 'Sample Index');
ylabel(axImag, 'Amplitude');

axMag = nexttile;
magLine = animatedline(axMag, 'Color', [0.47, 0.67, 0.19]);
title(axMag, 'CIR Magnitude');
xlabel(axMag, 'Sample Index');
ylabel(axMag, 'Magnitude');

drawnow limitrate;

samplesRead = 0;
indices = zeros(numSamples, 1);
realVals = zeros(numSamples, 1);
imagVals = zeros(numSamples, 1);
magVals = zeros(numSamples, 1);

while samplesRead < numSamples
    rawLine = readline(sp);
    if rawLine == ""
        continue;
    end
    tokens = split(strtrim(rawLine), ',');
    if numel(tokens) < 4
        warning("Skipping malformed row: %s", rawLine);
        continue;
    end

    idx = str2double(tokens(1));
    realVal = str2double(tokens(2));
    imagVal = str2double(tokens(3));
    magVal = str2double(tokens(4));

    if any(isnan([idx, realVal, imagVal, magVal]))
        warning("Skipping row containing NaN values: %s", rawLine);
        continue;
    end

    samplesRead = samplesRead + 1;
    indices(samplesRead) = idx;
    realVals(samplesRead) = realVal;
    imagVals(samplesRead) = imagVal;
    magVals(samplesRead) = magVal;

    addpoints(realLine, idx, realVal);
    addpoints(imagLine, idx, imagVal);
    addpoints(magLine, idx, magVal);
    drawnow limitrate;
end

% Trim arrays in case malformed rows were skipped
indices = indices(1:samplesRead);
realVals = realVals(1:samplesRead);
imagVals = imagVals(1:samplesRead);
magVals = magVals(1:samplesRead);

% Final draw and autoscale after capture
for ax = [axReal, axImag, axMag]
    axis(ax, 'tight');
    grid(ax, 'on');
end

if isvalid(figureHandle)
    drawnow;
end

fprintf('Captured %d samples from %s at %d baud.\n', samplesRead, serialPort, baudRate);

captureTable = table(indices, realVals, imagVals, magVals, ...
    'VariableNames', {'Index', 'Real', 'Imag', 'Magnitude'});

clear sp;

    function cleanupSerial()
        if exist('sp', 'var') && ~isempty(sp)
            try
                flush(sp);
            catch
                % Ignore flush errors during cleanup
            end
        end
    end

end
