% Housekeeping
clc;
clear;
close all;

% Calculate size of frame from pixels and bits [bits]
c_360 = (24 * 360)/8000;
c_480 = (24 * 480)/8000;
c_720 = (24 * 720)/8000;
c_1080 = (24 * 1080)/8000;

% Component latencies [s]
gs_computer_latency = 93E-3;
flight_controller_latency = 150E-3;
companion_computer_latency = 101E-3;

% GS Computer Vision latency, this is 1 way, i.e does not
% get sent back to the UAS [s]
camera_latency_360p = 16.875E-3;
camera_latency_480p = 22.5E-3;
camera_latency_720p = 33.75E-3;
camera_latency_1080p = 50.625E-3;


% Assume 30 fps is good enough
% fps = 1:45;


% GS Motion Planning Latency, this is 2 way, i.e sends
% the UAS state to the GS and then GS sends back motion
% planning [s]

% Size of the data packet assuming 64 byte packet, worst case
n = 50;
suggested_bandwidth = 64; %kB/s
packet_size = (48:n*96)./1000; %kB
motion_planning_latency = packet_size./suggested_bandwidth; %s




O_O_O_latency = linspace(0,0,4);

O_O_G_latency = companion_computer_latency+gs_computer_latency+flight_controller_latency+motion_planning_latency;

O_G_O_latency_360p = camera_latency_360p + gs_computer_latency+companion_computer_latency;
O_G_O_latency_480p = camera_latency_480p + gs_computer_latency+companion_computer_latency;
O_G_O_latency_720p = camera_latency_720p + gs_computer_latency+companion_computer_latency;
O_G_O_latency_1080p = camera_latency_1080p + gs_computer_latency+companion_computer_latency;

O_G_O_latency = [O_G_O_latency_360p;O_G_O_latency_480p;O_G_O_latency_720p;O_G_O_latency_1080p];


O_G_G_latency_360p = camera_latency_360p+gs_computer_latency+flight_controller_latency+motion_planning_latency;
O_G_G_latency_480p = camera_latency_480p+gs_computer_latency+flight_controller_latency+motion_planning_latency;
O_G_G_latency_720p = camera_latency_720p+gs_computer_latency+flight_controller_latency+motion_planning_latency;
O_G_G_latency_1080p = camera_latency_1080p+gs_computer_latency+flight_controller_latency+motion_planning_latency;

O_G_G_latency = [O_G_G_latency_360p;O_G_G_latency_480p;O_G_G_latency_720p;O_G_G_latency_1080p];

data_size_O_O_O = [c_360;c_480;c_720;c_1080];
data_size_O_G_O = [c_360;c_480;c_720;c_1080];

data_size_O_G_G_360p = c_360+motion_planning_latency;
data_size_O_G_G_480p = c_480+motion_planning_latency;
data_size_O_G_G_720p = c_720+motion_planning_latency;
data_size_O_G_G_1080p = c_1080+motion_planning_latency;

% Plot the different data processing methods
processing_methods_strings = ["O-O-O","O-O-G","O-G-O 360p","O-G-O 480p",...
                              "O-G-O 720p","O-G-O 1080p","O-G-G 360p", ...
                              "O-G-G 480p","O-G-G 720p","O-G-G 1080p"];
latency_maxes = [max(O_O_O_latency)*1000,max(O_O_G_latency)*1000,max(O_G_O_latency_360p)*1000, ...
                 max(O_G_O_latency_480p)*1000,max(O_G_O_latency_720p)*1000, max(O_G_O_latency_1080p)*1000, ...
                 max(O_G_G_latency_360p)*1000, max(O_G_G_latency_480p)*1000,max(O_G_G_latency_720p)*1000, ...
                 max(O_G_G_latency_1080p)*1000];

% Create a numeric array for x-axis
x = 1:numel(processing_methods_strings);

% Create the bar plot with custom tick labels
bar(x, latency_maxes);
set(gca, 'XTick', x);
set(gca, 'XTickLabel', processing_methods_strings);
xtickangle(45); % Optional: Rotate x-axis labels for better readability
xlabel("Data Processing Method");
ylabel("Latency (Worst Case) [ms]")
hold on;


% figure;
% bar("O-O-G",O_O_G_latency(end)*1000);
% xlabel("Packet Size")
% ylabel("Latency [s]")

% figure;
% plot(data_size_O_G_O,O_G_O_latency*1000);
% xlabel("Packet Size")
% ylabel("Latency [s]")

% figure;
% plot(data_size_O_G_G_360p,O_G_G_latency_360p*1000);
% xlabel("Packet Size")
% ylabel("Latency [s]")

% figure;
% plot(data_size_O_G_G_480p,O_G_G_latency_480p*1000);
% xlabel("Packet Size")
% ylabel("Latency [s]")

% figure;
% plot(data_size_O_G_G_720p,O_G_G_latency_720p*1000);
% xlabel("Packet Size")
% ylabel("Latency [s]")

% figure;
% plot(data_size_O_G_G_1080p,O_G_G_latency_1080p*1000);
% xlabel("Packet Size")
% ylabel("Latency [s]")


% hold off
% 
% xlabel("Packet Size")
% ylabel("Latency [s]")
% legend("O-O-O","O-O-G","O-G-O", "O-G-G 360p","O-G-G 480p","O-G-G 720p","O-G-G 1080p","Location","best")
% title("Data Latency for Different Data Processing Methods (Worst Case)")

% colorOrder = [
%     0, 0.4470, 0.7410;   % Blue
%     0.8500, 0.3250, 0.0980;  % Red
%     0.9290, 0.6940, 0.1250;  % Yellow
%     0.4940, 0.1840, 0.5560;  % Purple
%     0.4660, 0.6740, 0.1880;  % Green
%     0.3010, 0.7450, 0.9330;  % Light Blue
%     0.6350, 0.0780, 0.1840;  % Dark Red
%     0, 0, 0;  % Black
%     0.25, 0.25, 0.25;  % Gray
%     0.5, 0.25, 0;  % Brown
% ];


% % Set the color order for the current axes
% set(gca, 'ColorOrder', colorOrder);


% Create a table with the specified data
% data = {O_O_O_latency(4);
%         O_O_G_latency(4);
%         O_G_O_latency(4);
%         O_G_G_latency(4)};

% labels = {'O-O-O','O-O-G', 'O-G-O','O-G-G'};
% 
% % Create a table with cell data types
% T = table(labels', data, 'VariableNames', {'Data Processing Mode', 'Latency at 30 FPS (worst case) [s]'});
% 
% % Display the table
% display(T)