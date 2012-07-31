% graphics_toolkit ("fltk")

sprintf("Accels Mean: %f %f %f", mean(accels))
sprintf("Gyros Mean: %f %f %f", mean(gyros))
sprintf("Mags Mean: %f %f %f", mean(mags))

c = ones(length(accels),1);

caccels = accels - c*mean(accels);
cgyros = gyros - c*mean(gyros);
cmags = mags - c*mean(mags);

figure(1);
subplot(3,1,1); plot(caccels); ylabel("Accels (g's)");
subplot(3,1,2); plot(cgyros); ylabel("Gyros (rads/s)");
subplot(3,1,3); plot(cmags); ylabel("Mags (?)");

Fs = 20;
y = caccels(:,1);
L = length(y);
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(y,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

% Plot single-sided amplitude spectrum.
figure(2);
plot(f,2*abs(Y(1:NFFT/2+1))) 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')

figure(3); 
y = caccels;
L = length(y);
NFFT = 2^nextpow2(L);
plot(linspace(-0.5,0.5,NFFT),fftshift(abs(fft(y,NFFT)).^2));
