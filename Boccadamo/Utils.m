Kriv = 5000;

Mope = 4;

Mrot = 1.2 ;

M1 = Mrot/2;
M2 = Mrot/2;

Mlink = 0.10005 ;

k = 1 ;
L0 = 0.7*pi ;
l0 =0;

init_pos= -2*pi;

initState = [init_pos-L0*sqrt(Ktrasm/k)/(2*sqrt(Ktrasm/k)+1),...
    init_pos,...
    init_pos+L0*sqrt(Ktrasm/k)/(2*sqrt(Ktrasm/k)+1),...
    0, 0, 0];

Umax = 30 ;
U1max = Umax /2;
U2max = Umax /2;
HICmax = 100 ;


save funUTILS.mat M1 M2 Mlink Ktrasm k L0 l0 initState
HICmax Mope Kriv Ttobeat granularity N;