clc; clear all; close all;

data = load('dob_data.txt');

figure
set(gcf,'Color',[1,1,1])
subplot(311)
plot(data(:,1), data(:,2));
grid on
xlim([0 14])

subplot(312)
plot(data(:,1), data(:,3));
grid on
xlim([0 14])

subplot(313)
plot(data(:,1), data(:,5));
grid on
xlim([0 14])