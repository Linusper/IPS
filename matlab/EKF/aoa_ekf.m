clear all; close all; clc
len = 2;


fallet = "1.b";
measure = 1;
cutoff = 0.05;

if measure == 1
    if fallet == "1.f"
        riktigtX = 0.43;
        riktigtY = 0.61;
        T = readtable('angle_samples.csv');
        %T = readtable('fall1_2.csv');
    elseif fallet == "1.b"
        riktigtX = 2.35;
        riktigtY = 3.935;
        T = readtable('Data\100\rtls_angle_samples.csv');
        %T = readtable('fall1_bak2.csv');
    elseif fallet == "2.f"
        riktigtX = -0.77;
        riktigtY = 0.48;
        T = readtable('measure1\fall2_1.csv');
        %T = readtable('fall2_2.csv');
    elseif fallet == "2.b"
        riktigtX = -0.80;
        riktigtY = 1.5;
        T = readtable('measure1\fall2_bak1.csv');
        %T = readtable('fall2_bak2.csv');
    elseif fallet == "3.f"
        riktigtX = 2.0;
        riktigtY = 0.8;
        T = readtable('measure1\fall3_1.csv');
        %T = readtable('fall3_2.csv');
    elseif fallet == "3.b"
        riktigtX = 2.0;
        riktigtY = 2.0;
        T = readtable('measure1\fall3_bak1.csv');
        %T = readtable('fall3_bak2.csv');
    end
    measurementTime = 60;
    [leftAng, rightAng] = table2array(T);
elseif measure == 2
    if fallet == "1.f"
        riktigtX = 0.47;
        riktigtY = 1.54;
        T = readtable('measure2\fall1_ny2.csv');
    elseif fallet == "2.f"
        riktigtX = -1.03;
        riktigtY = 1.13;
        T = readtable('measure2\fall2_ny2.csv');
    elseif fallet == "3.f"
        riktigtX = 1.96;
        riktigtY = 1.07;
        T = readtable('measure2\fall3_ny2.csv');
    end
    measurementTime = 60;
    [rightAng, leftAng] = table2array(T);
end

% T = readtable('measure2\fall_mega.csv');
%[rightAng, leftAng] = table2array(T);

leftAng = leftAng(5:end);
rightAng = rightAng(5:end);
if length(leftAng) > length(rightAng)
    leftAng = leftAng(1:length(rightAng));
elseif length(rightAng) > length(leftAng)
    rightAng = rightAng(1:length(leftAng));
end
% leftAng = leftAng-0.25*sqrt(riktigtX^2+riktigtY^2);
% rightAng = rightAng+0.25*sqrt(riktigtX^2+riktigtY^2);

if strncmpi(fallet,"1",1)
    riktigtA = atand(riktigtY/riktigtX);
    riktigtB = atand(riktigtY/abs(len - riktigtX));
elseif strncmpi(fallet,"2",1)
    riktigtA = 180 - atand(riktigtY/abs(riktigtX));
    riktigtB = atand(riktigtY/(len + abs(riktigtX)));
elseif strncmpi(fallet,"3",1)
    riktigtA = atand(riktigtY/(abs(riktigtX)));
    riktigtB = 180 - atand(riktigtY/(riktigtX - len));
end


n = measurementTime/length(leftAng);

position = [];
allaA = [];
allaB = [];
allaC = [];

fall1 = 0;
fall2 = 0;
fall3 = 0;

nrSkit = 0;

for i = 1:length(leftAng)
    [curPos, fall, A, B, C] = triangulering(leftAng(i),rightAng(i),len);
    
    if isnan(curPos)
        nrSkit = nrSkit + 1;
        continue
    end
    
    switch fall
        case 1
            fall1 = fall1 + 1;
        case 2
            fall2 = fall2 + 1;
        case 3
            fall3 = fall3 + 1;
    end
    
    position = [position; curPos];
    allaA = [allaA; A];
    allaB = [allaB; B];
    allaC = [allaC; C];
end
t = linspace(0,measurementTime,length(position));

window = 10;
meanLeftAng = movmean(leftAng,window);
meanRightAng = movmean(rightAng,window);

allaAef = [];
allaBef = [];
allaCef = [];

% Fs = length(leftAng)/(measurementTime);
% leftAngEf = lowpass(leftAng,cutoff,Fs);
% rightAngEf = lowpass(rightAng,cutoff,Fs);
% %leftAngEf = lowpass(meanLeftAng,cutoff,Fs);
% %rightAngEf = lowpass(meanRightAng,cutoff,Fs);
Ts = 0.1;
[filtLeft,filtRight] = filtering(leftAng,rightAng,Ts);

positionEf = [];
nrSkitEf = 0;
for i = 1:length(filtLeft)
    [curPosEf, ~, Aef, Bef, Cef ] = triangulering(filtLeft(i),filtRight(i),len);
    
    if isnan(curPosEf)
        nrSkitEf = nrSkitEf + 1;
        continue
    end
    
    positionEf = [positionEf;curPosEf];
    allaAef = [allaAef; Aef];
    allaBef = [allaBef; Bef];
    allaCef = [allaCef; Cef];
end


T1 = Ts * [0:length(leftAng)-1]';  % time f�r simulink
%[filtLeft,filtRight] = filtering(leftAng,rightAng,Ts);
posLeft = positionEf(:,1);
posRight = positionEf(:,2);
aoa_data = [T1 posLeft posRight];

% figure
% plot(T1,leftAngEf, T1, rightAngEf, T1, leftAng, T1, rightAng)
% legend('left ang lowpass', 'right ang lowpass', 'left ang', 'right ang')
%%
a = sim('aoa_ekfmodel', 'SimulationMode', 'normal');
b = a.get('xpos_ekf');
c = a.get('ypos_ekf');
assignin('base', 'b',b);
assignin('base','c',c);
    
figure
plot(T1, posLeft, T1, posRight)
legend('v�nster', 'h�ger')

meanb = mean(b);
meanc = mean(c);
figure
hold on
plot(1, 1, 'o')
plot(meanb, meanc, '*')
plot(b,c, '*')
hold off

figure
plot(T1, filtLeft, T1, filtRight,T1, leftAng, T1, rightAng)
legend('left filt', 'right filt','left ang', 'right ang')
