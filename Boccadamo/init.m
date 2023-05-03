clear;
close all;
clc;

% Kvect contains multiple values for K_trasmission

Kvect = [];
tauvect = [];

Kvect = [Kvect [1E-3:1E-3:1E-2]];
tauvect = [tauvect 0.16*ones(1,10)];
 
Kvect = [Kvect [0.02:0.01:0.1]];
tauvect = [tauvect 0.05*ones(1,9)];
 
Kvect = [Kvect [0.2:0.1:1]];
tauvect = [tauvect 0.04*ones(1,9)];
 
Kvect = [Kvect [2:1:10]];
tauvect = [tauvect 0.03*ones(1,9)];
 
Kvect = [Kvect [20:10:100]];
tauvect = [tauvect 0.04*ones(1,9)];
 
Kvect = [Kvect [2E2:1E2:1E3]];
tauvect = [tauvect 0.04*ones(1,9)];


 
