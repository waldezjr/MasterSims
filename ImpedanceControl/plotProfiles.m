clear all;
close all;
clc;
clf;
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

x = linspace(0,1)
y = -x

plot(x,y)
xlim([-.25 1.25])
ylim([0.25 -1.25])