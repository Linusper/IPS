%close all; clear all; clc;
% ACC
acc_samples = readtable('Data\rtls_acc_samples.csv');
X= [];
Y= [];
Z = [];
static = 0.5;
Ts = 0.25;

for i = 1:length(acc_samples.X)           
  X = [X; acc_samples.X(i)*3.9*9.82/1000];
  Y = [Y; acc_samples.Y(i)*3.9*9.82/1000];
  Z = [Z; acc_samples.Z(i)];
end

for i = 1:length(X)
    if abs(X(i)) < static
        X(i) = 0;
    end
    if abs(Y(i)) < static
        Y(i) = 0;
    end
end

    for i = 1:length(X)
        if Z(i) < 230 || Z(i) > 280
            if i == 1
                X(i) = X(i+1);
                Y(i) = Y(i+1);
                Z(i) = Z(i+1);
            elseif i == length(X)
                X(i) = X(i-1);
                Y(i) = Y(i-1);
                Z(i) = Z(i-1);
            else
                X(i) = (X(i-1)+X(i+1))/2;
                Y(i) = (Y(i-1)+Y(i+1))/2;
                Z(i) = (Z(i-1)+Z(i+1))/2;
            end
        end
    end

[X_filt,Y_filt] = filtering(X,Y,Ts);

xvel = cumtrapz(Ts,X_filt);
yvel = cumtrapz(Ts,Y_filt);

xpos = cumtrapz(Ts, xvel);
ypos = cumtrapz(Ts, yvel);

T1 = Ts * [0:length(xvel)-1]';  % time för simulink

acc_data = [T1 X_filt Y_filt];
vel_data = [T1 xvel yvel];
figure
plot(T1, xvel,T1, yvel, T1, X_filt, T1, Y_filt)
legend('X velocity', 'Y velocity', 'X acc', 'Y acc')

figure 
plot(T1, X, T1, Y)
legend('X raw', 'Y raw')

figure
plot(T1, xpos, T1, ypos)
legend('xpos', 'ypos')

figure
plot(xpos, ypos)
legend('x loc', 'y loc')