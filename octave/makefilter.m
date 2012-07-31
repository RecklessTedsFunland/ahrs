function makefilter(b,a, data)

%c = cut/(sample/2);
%w = linspace(0,1,sample/2);
%atten = 20; %dB of attenuation in stop band

%[b,a]=butter ( n, c );
%[b,a]=cheby2(n,atten,c);
filtered = filter(b,a,data);

printf("Mean, STD: %f %f \n",mean(data),std(data));

close all;
[h,w]=freqz(b,a);
figure(1); freqz(b,a);
figure(2);
subplot(2,1,1); plot(data); ylabel('orig');
subplot(2,1,2); plot(filtered); ylabel('filtered');

sys = tf2sys(b,a);
[v,t]=step(sys);
figure(3); plot(t,v);
