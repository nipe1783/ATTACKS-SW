clc
clear
close all
%Data transmission latency
image_size = [360, 480, 720, 1080];
byte_per_pixel = 3;
packet_size = image_size .* byte_per_pixel ./ 1000; %kB
suggested_bandwidth = 64; %kB/s
frames = 1:30;

n = 50;
packet_size = (48:n*96)./1000; %kB
latency = 1000*packet_size./suggested_bandwidth; %ms

figure()
hold on

plot(packet_size,latency,'LineWidth',2)
scatter([min(packet_size), max(packet_size)],[min(latency), max(latency)],'f','LineWidth',2)

text([min(packet_size), max(packet_size)], [min(latency), max(latency)], sprintfc(' %.2f',[min(latency), max(latency)]),'Vert','top', 'FontSize',11)

xlabel('Packet Size (kB)')
ylabel('Latency (ms)')
title('Latency of Transmitting a Path')


figure()
hold on
for image_size = [360, 480, 720, 1080]
   packet_size = image_size .* frames .* byte_per_pixel ./ 1000; %kB
   latency = 1000*packet_size./suggested_bandwidth; %ms
   plot(frames,latency)
end
xlabel('Number of frames in packet')
ylabel('Latency (ms)')
title('Latency for Image Transmission Packet Sizes')
legend('360p', '480p', '720p', '1080p')

figure()
hold on
image_size = [360, 480, 720, 1080];
packet_size = image_size .* 1 .* byte_per_pixel ./ 1000; %kB
latency = 1000*packet_size./suggested_bandwidth; %ms

plot(image_size,latency,'LineWidth',2)
scatter(image_size,latency,'f','LineWidth',2)

text(image_size, latency, sprintfc(' %.3f',latency),'Vert','top', 'FontSize',11)

xlabel('Image size (p)')
ylabel('Latency (ms)')
title('Latency of Transmitting an Image at Various Sizes')


