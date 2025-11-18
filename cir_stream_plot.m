% cir_stream_plot.m
% Stream and plot DW3000 CIR data coming from the Arduino RX sketch.
% Update the serial port string below to match your system.

port = "COM3";        % <-- change to the correct port (e.g., "/dev/ttyACM0")
baud = 115200;        % Must match the Arduino sketch

if ~any(serialportlist == port)
    error("Serial port %s not found. Update the 'port' variable.", port);
end

sp = serialport(port, baud);
configureTerminator(sp, "LF");
flush(sp);

fig = figure('Name', 'DW3000 CIR Streaming');

buffering = false;
frameLines = string.empty(0, 1);

while ishandle(fig)
    line = strtrim(readline(sp));

    if line == "FRAME_BEGIN"
        buffering = true;
        frameLines = string.empty(0, 1);
        continue;
    elseif line == "FRAME_END"
        if buffering && ~isempty(frameLines)
            % Parse buffered sample lines
            idx = [];
            realPart = [];
            imagPart = [];
            mag = [];

            for k = 1:numel(frameLines)
                parts = split(frameLines(k), ',');
                if numel(parts) ~= 4
                    continue; % Ignore malformed lines
                end
                vals = str2double(parts);
                if any(isnan(vals))
                    continue;
                end
                idx(end+1) = vals(1); %#ok<AGROW>
                realPart(end+1) = vals(2); %#ok<AGROW>
                imagPart(end+1) = vals(3); %#ok<AGROW>
                mag(end+1) = vals(4); %#ok<AGROW>
            end

            if ~isempty(idx)
                clf(fig);
                subplot(3, 1, 1);
                plot(idx, realPart, '-b');
                ylabel('Amplitude');
                title('DW3000 CIR Streaming');

                subplot(3, 1, 2);
                plot(idx, imagPart, '-r');
                ylabel('Amplitude');

                subplot(3, 1, 3);
                plot(idx, mag, '-k');
                ylabel('Amplitude');
                xlabel('Sample index');
                drawnow;
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
