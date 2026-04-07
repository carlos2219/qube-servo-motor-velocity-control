%Instituto Tecnológico y de Estudios Superiores de Monterrey
%Name: Carlos Hernán Auquilla Larriva
%Course: Autonomy of Unmanned Aerial Vehicles
%Last update: September 2nd, 2024
clc; clear;close all;

%Practice 6B PI and I+P control strategies

%%System parameters
rm = 7.5;
kt = 0.0422;
km = 0.0422;
jm = 1.4e-6;
lm = 1.15e-3;
mh = 0.0106;
rh = 0.0111;
md = 0.053;
rd = 0.0248;
u = -2e-3;  %Qube servo 3, SN: 56856
jh = 0.5 * mh * rh^2;
jd = 0.5 * md * rd^2;
jeq = jm + jh + jd;

%Open-Loop Transfer Fuction
num_op = kt/(jeq*rm);
den_op = [1 (km + u)*(kt/(jeq*rm)) 0];
G_op = tf(num_op,den_op);

%%Desired response
tp_d = 0.15;
MP_d = 2.5/100;

zeta_d = sqrt(( log(MP_d) )^2 / (( log(MP_d) )^2 + pi^2));
wd_d = pi /tp_d;
wn_d = wd_d / sqrt(1-zeta_d^2);
den_d = [1 2*wn_d*zeta_d wn_d^2]; %Ch. equation
sigma_d = zeta_d*wn_d;

%Controller gains
%D.9
ki=wn_d^2*jeq*rm/kt;
kp=(2*zeta_d*wn_d*jeq*rm/kt)-km-u;

%Obtained data
%Original controller
A_ob=2.407;
B_ob=4e1;
MP_ob=A_ob/B_ob;%The controller by itself does not meet the MP requirement.
tp_ob=0.141613;

%Modified controller for Kp and Ki
%Kp = 0.16
%Ki=4
A_mod=9.947e-1;
B_mod=4e1;
MP_mod=A_mod/B_mod;
tp_mod=0.147248;

