% 2DX3 Studio 9 - Exercise 2 - MATLAB Serial Communication
%
% Description:
%   Establishes UART communication between PC (MATLAB) and MCU.
%   PC sends the start character 's' to the MCU to trigger transmission.
%   MCU then sends 10 rounds of measurement data as formatted strings,
%   each terminated with \r\n (carriage return + newline).
%   MATLAB reads and prints each full line to the Command Window.
%
%   MCU sends lines in the format:
%   "measurement #(i), data1 (x), data2 (x), data3 (x)\r\n"
%
% MCU Baud Rate: 115200 bps (PIOSC 16MHz, IBRD=8, FBRD=44)
%
% Usage:
%   1. Flash the 2DX_2022_Studio_E2 Keil project onto your MCU.
%   2. Update the 'port' variable below to match your COM port.
%      - Windows: "COM3", "COM4", etc.
%      - Mac:     "/dev/cu.usbmodemXXXXXX"
%      Run >> serialportlist    in MATLAB Command Window to find your port.
%   3. Run this script.

%% Configuration 
port     = "COM5";       % <-- CHANGE THIS to your MCU's serial port
baudrate = 115200;       % Must match MCU UART_Init() setting

num_scans = 3;          % number of scans
num_angles    = 32;      % measurements per scan (every 11.25 degrees)
x_spacing     = 0.10;    % 20 cm between scans (pretend displacement)

%% Open the serial port 
device = serialport(port, baudrate);
device.Timeout = 120;     % 120 second read timeout

% Configure line terminator to match MCU's \r\n output
% readline() will read until it sees a newline (LF = "\n")
configureTerminator(device, "CR/LF");

fprintf("Opening: %s\n", port);

%% Flush / reset the buffers
flush(device);

%% Wait for user to press Enter 
% input("Press Enter to start communication...");
input("Press Enter then press button on MCU...");

%% Send start flag 's' to MCU via UART
write(device, 's', "char");

%% Receive 10 lines of measurement data from MCU 
% fprintf("\nReceived measurements:\n");
% for i = 1:10
%    x = readline(device);   % Read one full line
%    fprintf("%s\n", x);
% end

%% Receive data
angles_deg = (0:num_angles-1) * 11.25;
angles_rad = deg2rad(angles_deg);

X = zeros(num_scans, num_angles);
Y = zeros(num_scans, num_angles);
Z = zeros(num_scans, num_angles);
x_positions = (0:num_scans-1) * x_spacing;

fprintf("Receiving data...\n");

for s = 1:num_scans
    % flush any status messages from MCU before reading data
    % if s > 1
    %     pause(0.5);
    %     flush(device);
    % end
    for a = 1:num_angles
        line = readline(device);
        parts = str2double(strsplit(line, ','));
        dist_m = parts(3) / 1000.0;  % convert mm to m

        X(s, a) = x_positions(s);
        Y(s, a) = dist_m * cos(angles_rad(a));
        Z(s, a) = dist_m * sin(angles_rad(a));

        fprintf("Scan %d, Angle %.2f deg, Dist %.4f m\n", ...
                s, angles_deg(a), dist_m);
    end
end

% end data collection and clear device
% fprintf("Data collection complete.\n");
% clear device;

%% Flatten for scatter3
C = zeros(num_scans * num_angles, 1);
for p = 1:num_scans
    idx = (p-1)*num_angles + 1 : p*num_angles;
    C(idx) = p;
end

Xf = X(:); Yf = Y(:); Zf = Z(:);

%% Plot A: scatter3
figure('Name', 'G3 ToF 3D Scan');

subplot(1, 2, 1)
scatter3(Xf, Yf, Zf, 60, C, 'filled')
colormap(gca, jet(num_scans))
cb = colorbar; cb.Ticks = 1:num_scans;
cb.TickLabels = arrayfun(@(x) sprintf('x=%.2fm', x), x_positions, ...
                         'UniformOutput', false);
xlabel('X – displacement (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('scatter3')
axis equal; grid on; view(35, 25)
xlim([-0.5, 2]);
ylim([-3, 3]);
zlim([-3, 3])

%% Plot B: scan rings connected with lines
subplot(1, 2, 2)
hold on
cmap = jet(num_scans);

for p = 1:num_scans
    y_loop = [Y(p,:), Y(p,1)];
    z_loop = [Z(p,:), Z(p,1)];
    x_loop = repmat(x_positions(p), 1, num_angles+1);
    plot3(x_loop, y_loop, z_loop, '-o', ...
          'Color', cmap(p,:), 'LineWidth', 2, ...
          'MarkerSize', 5, 'MarkerFaceColor', cmap(p,:));

    if p > 1
        for a = 1:num_angles
            plot3([x_positions(p-1), x_positions(p)], ...
                  [Y(p-1,a), Y(p,a)], ...
                  [Z(p-1,a), Z(p,a)], ...
                  '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
        end
    end
end

% plot details
xlabel('X – displacement (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Scan rings + connections')
axis equal; grid on; view(35, 25)
xlim([-0.5, 2]);
ylim([-3, 3]);
zlim([-3, 3]) % limits can be changed depending on location scanned

%% Close the serial port 
fprintf("Closing: %s\n", port);
clear device;