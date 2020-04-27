close all; clear all; clc;
% ACC
acc_samples = readtable('acc_samples_rull.csv');
X= [];
Y= [];
static = 0.5;

for i = 1:length(acc_samples.X)            %35 första samples 
  X = [X; acc_samples.X(i)*3.9*9.82/1000];
  Y = [Y; acc_samples.Y(i)*3.9*9.82/1000];
end

for i = 1:length(X)
    if abs(X(i)) < static
        X(i) = 0;
    end
    if abs(Y(i)) < static
        Y(i) = 0;
    end
end

Fs = 5;                            % Sampling Frequency (Hz)
Fn = Fs/2;                          % Nyquist Frequency (Hz)
Fco = 0.3;                            % Cutoff Frequency (Hz)
Wp = Fco/Fn;                        % Normalised Cutoff Frequency (rad)
Ws = 1.2*Wp;                        % Stopband Frequency (rad)
Rp =  0.3;                            % Passband Ripple (dB)
Rs = 1;                            % Stopband Ripple (dB)
[n,Wn]  = buttord(Wp,Ws,Rp,Rs);     % Calculate Filter Order
[b,a]   = butter(n,Wn);             % Calculate Filter Coefficients
[sos,g] = tf2sos(b,a);              % Convert To Second Order Section Representation
figure(1)
freqz(sos, 1024, Fs);                % Plot Filter Response (Bode Plot)

dt = 0.25;
t = (1:length(X))*dt;

filt_signalX = filtfilt(sos, g, X);    % Filter Signal
filt_signalY = filtfilt(sos, g, Y);    % Filter Signal

xvel = cumtrapz(dt,filt_signalX);
yvel = cumtrapz(dt,filt_signalY);
T1 = dt * [0:length(xvel)-1]';  % time för simulink

acc_data = [T1 xvel yvel];
plot(T1, xvel,T1, yvel)

%% AOA
T = readtable('angle_samples2.csv');
%Ang = table2array(T(:,4));
leftAng = [];
rightAng = [];

xpos_L = [];
xpos_R = [];
ypos_R = [];
ypos_L = [];
len = 0.92;

for i = 1:length(T.name)   
    if char(T.name(i)) == 'CC2640r2 Passive V'
        leftAng = [leftAng; T.angle(i)];
    elseif char(T.name(i)) == 'CC2640r2 Passive H'
        rightAng = [rightAng; T.angle(i)];
    end
end
nrSkit = 0

windowSize = 3; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
filter_right = filter(b,a,rightAng);
filter_left = filter(b,a,leftAng);

for i = 5:length(filter_left) % 5:length(filter_left)

    if filter_left(i) <= 0 && filter_right(i) >= 0
        %disp('Fall 1');
        A = 90 - abs(filter_left(i));
        B = 90 - abs(filter_right(i));
        C = 180 - A - B;
        if C <= 0
            disp('sket sig')
            nrSkit = nrSkit + 1;
            continue
        end

        HL = (len * sin(deg2rad(A))) / sin(deg2rad(C));
        VL = (len * sin(deg2rad(B))) / sin(deg2rad(C));

        x1_r = HL * cos(deg2rad(B));
        
        ypos_R = [ypos_R, HL * sin(deg2rad(B))];
        xpos_R = [xpos_R, len - x1_r];
        
        xpos_L = [xpos_L, VL * cos(deg2rad(A))];
        ypos_L = [ypos_L, VL * sin(deg2rad(A))];
    
    elseif filter_left(i) > 0 && filter_right(i) > 0
        %disp('Fall 2');
        A = 90 + abs(filter_left(i));
        B = 90 - abs(filter_right(i));
        C = 180 - A - B;
        if C <= 0
            disp('sket sig')
            nrSkit = nrSkit + 1;
            continue
        end
        
        HL = (len * sin(deg2rad(A))) / sin(deg2rad(C));
        VL = (len * sin(deg2rad(B))) / sin(deg2rad(C));
        
        x1_r = HL * cos(deg2rad(B));
        
        ypos_R = [ypos_R, HL * sin(deg2rad(B))];
        xpos_R = [xpos_R, x1_r - len];
        
        xpos_L = [xpos_L, -1 * VL * cos(deg2rad(180 - A))];
        ypos_L = [ypos_L, VL * sin(deg2rad(180 - A))];
        
    elseif filter_left(i) < 0 && filter_right(i) < 0
        %disp('Fall 3');
        A = 90 - abs(filter_left(i));
        B = 90 + abs(filter_right(i));
        C = 180 - A - B;
        if C <= 0
            disp('sket sig');
            nrSkit = nrSkit + 1;
            continue
        end
        
        HL = (len * sin(deg2rad(A))) / sin(deg2rad(C));
        VL = (len * sin(deg2rad(B))) / sin(deg2rad(C));
        
        x1_r = HL * cos(deg2rad(180 - B));
        
        ypos_R = [ypos_R, HL * sin(deg2rad(180 - B))];
        xpos_R = [xpos_R, x1_r + len];
        
        xpos_L = [xpos_L, VL * cos(deg2rad(A))];
        ypos_L = [ypos_L, VL * sin(deg2rad(A))];
    end
end
xpos = xpos_L';
ypos = ypos_L';
dt = 0.10;
T2 = dt * [0:length(xpos)-1]';  % time för simulink

aoa_data = [T2 xpos ypos];
plot(T2, xpos,T2, ypos)
