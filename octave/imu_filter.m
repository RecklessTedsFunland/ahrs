Fs = 20;
Wp = .8; Ws = .9;
Rp = 1; Rs = 40;
[n,Ws] = cheb2ord(Wp,Ws,Rp,Rs);
[b,a] = cheby2(n,Rs,Ws);

freqz(b,a); 
tt=sprintf("n=%d Chebyshev Type II Lowpass Filter [Wp = %d Hz Ws = %d Hz]",n,Wp*Fs/2,Ws*Fs/2);
title(tt);
