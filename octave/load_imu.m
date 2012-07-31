clear all;
close all;

graphics_toolkit ("fltk")
pkg load signal

m=dlmread("test.txt",",");
n = m(:,5:13);
% accels gyros mags
accels = n(10:length(m),1:3);
gyros = n(10:length(m),4:6);
mags = n(10:length(m),7:9);

accels = accels - ones(length(accels),1)*[0 0 1];

sprintf("Accels Mean: %f %f %f", mean(accels))
sprintf("Gyros Mean: %f %f %f", mean(gyros))
sprintf("Mags Mean: %f %f %f", mean(mags))
