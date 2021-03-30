%% experimental calculations 30.03.2021

L=16e-3;    %coil inductance
R=7.8;      %resistance at 3kHz
omega_ideal=2*pi*3e3;
Cideal=1/(omega^2*L);
C=200e-9;       %next useful capacitance
omega=1/(sqrt(L*C));
vdc=30;         %dc link voltage
vin_ampl=4/pi*vdc;   %fundamental frequency amplitude of input voltage

imax_ampl=vin_ampl/R;
vmax_ampl=imax_ampl*omega*L;