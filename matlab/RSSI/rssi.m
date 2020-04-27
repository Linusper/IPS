% RSSI
clc; clear all; close all; 
master_rssi = readtable('EKF\Data\rtls_rssi_samples.csv');
angle_rssi = readtable('EKF\Data\100\rtls_angle_samples.csv');
kalibrering = readtable('EKF\Data\rssi\rtls_rssi_kalibrering.csv');

% Calibration setup
Kalibrering = table2array(kalibrering);
A = mean(Kalibrering);
n = 2;
d = 2;

%Read RSSI values
leftRssi = [];
rightRssi = [];
masterRssi = table2array(master_rssi);


for i = 1:length(angle_rssi.name)   
    if char(angle_rssi.name(i)) == 'CC2640r2 Passive V'
        leftRssi = [leftRssi; angle_rssi.rssi(i)];
    elseif char(angle_rssi.name(i)) == 'CC2640r2 Passive H'
        rightRssi = [rightRssi; angle_rssi.rssi(i)];
    end
end

leftRssi = smooth(leftRssi,5,'sgolay',1);
rightRssi = smooth(rightRssi,5,'sgolay',1);
masterRssi = smooth(masterRssi,5,'sgolay',1);


%% Calculate n for environment, n range 2-4
d=1000
for j = 1:length(masterRssi)
    n_master(j) = -((masterRssi(j) + A)/(10*log10(d)));
end
n_master = mean(n_master);

for i = 1:length(leftRssi)
    n_left(i) = -((leftRssi(i) + A)/(10*log10(d)));
    n_right(i) = -((rightRssi(i) + A)/(10*log10(d)));
end
n_left = mean(n_left);
n_right = mean(n_right);
%n_master = 2;
% n_left = n_master;
% n_right = n_master;

%% Estimate distance
%RSSI = -(10nlog_10(d) + A)
for i = 1:length(masterRssi)
    r_master(i) = 10.^((A - masterRssi(i))/(10*n_master));
end

for i = 1:length(leftRssi)
    r_left(i) = 10.^((A - leftRssi(i))/(10*n_left));
    r_right(i) = 10.^((A - rightRssi(i))/(10*n_right));
end

r_master = mean(r_master);
r_left = mean(r_left);
r_right = mean(r_right);

%Distance = 10 ^ ((Measured Power – RSSI)/(10 * N))

%% Trilateration
% node positions
x_m = 1;
y_m = 0;
x_V =0;
y_V = 0;
x_H = 2;
y_H = 0;


[x1, y1] = circ(x_m,y_m,r_master,x_V,y_V,r_left);
[x2, y2] = circ(x_V,y_V,r_left, x_H, y_H, r_right);
[x3, y3] = circ(x_m,y_m,r_master,x_H,y_H,r_right);

[xint1,yint1] = polyxpoly(x1,y1,x2,y2)
[xint2,yint2] = polyxpoly(x1,y1,x3,y3)
[xint3,yint3] = polyxpoly(x3,y3,x2,y2)


% circlar
th = 0:pi/50:2*pi;
xm = r_master * cos(th) + x_m;
ym = r_master * sin(th) + y_m;
xV = r_left * cos(th) + x_V;
yV = r_left * sin(th) + y_V;
xH = r_right * cos(th) + x_H;
yH = r_right * sin(th) + y_H;

figure
hold on
plot(xm, ym)
plot(xV,yV)
plot(xH,yH)
plot(x1,y1)
plot(x2,y2)
plot(x3,y3)
%plot(xint1, yint1, '*')
%plot(xint2, yint2, 'o','MarkerSize', 10)
%plot(xint3, yint3, 'o', 'MarkerSize', 20)
plot(1, 1, '*')
plot(0,0, 'o')
plot(x_V, y_V, 'o')
plot(x_H, y_H, 'o')
%plot(x,y,'o','MarkerSize',sz)




%% Line intersect

l1 = [x1(1) y1(1) x1(2) y1(2)];
l2 = [x2(1) y2(1) x2(2) y2(2)];
l3 = [x3(1) y3(1) x3(2) y3(2)];
%lineintersect(l1,l2)

[xpos12,ypos12]=lineintersect(l1,l2);
[xpos13,ypos13]=lineintersect(l1,l3);
[xpos23,ypos23]=lineintersect(l2,l3);

% circlar
th = 0:pi/50:2*pi;
xm = r_master * cos(th) + x_m;
ym = r_master * sin(th) + y_m;
xV = r_left * cos(th) + x_V;
yV = r_left * sin(th) + y_V;
xH = r_right * cos(th) + x_H;
yH = r_right * sin(th) + y_H;

figure
hold on
plot(xm, ym)
plot(xV,yV)
plot(xH,yH)
plot(x1,y1)
plot(x2,y2)
plot(x3,y3)
plot(xpos12,ypos12, '*')
plot(-4, 1.015, 'o')
plot(xpos13,ypos13, '*')
plot(xpos23,ypos23, '*')
%plot(xint1, yint1, '-')
legend('Master', 'Left', 'Right', 'Master-Left', 'Right-Left', 'Master-Right', 'Estimated Pos', 'Real Pos')
hold off




