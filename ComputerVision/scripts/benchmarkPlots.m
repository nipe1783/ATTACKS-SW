%% Housekeeping
clc;
close all;
clear;

%% Load in Data
basicData = readmatrix('../benchmarking/data/dataBasic.csv');
dogData = readmatrix('../benchmarking/data/dataDoG.csv');
varyingData = readmatrix('../benchmarking/data/dataVarying.csv');

%% Calculate Averages
basicExecTime = basicData(:,1);
basicPixelError = basicData(:,2);

basicAvgExecTime = mean(basicExecTime);
basicAvgPixelError = mean(basicPixelError);

dogExecTime = dogData(:,1);
dogPixelError = dogData(:,2);

dogAvgExecTime = mean(dogExecTime);
dogAvgPixelError = mean(dogPixelError);

varyingExecTime = varyingData(:,1);
varyingPixelError = varyingData(:,2);

varyingAvgExecTime = mean(varyingExecTime);
varyingAvgPixelError = mean(varyingPixelError);

%% Plotting
x = categorical({'Basic','DoG','Varying'});
y = [basicAvgExecTime, dogAvgExecTime, varyingAvgExecTime];

figure();
bar(x,y);
title("Average Execution Time for Different Blob Detection Algorithms");
xlabel("Blob Detection Algorithms");
ylabel("Execution Time (ms)");

x = categorical({'Basic','DoG','Varying'});
y = [basicAvgPixelError, dogAvgPixelError, varyingAvgPixelError];

figure();
bar(x,y);
title("Average Pixel Error for Different Blob Detection Algorithms");
xlabel("Blob Detection Algorithms");
ylabel("Number of Incorrect Pixels");