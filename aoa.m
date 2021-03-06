clc; clear; close all;

len = 0.92;


fallet = "1.f";
measure = 2;
cutoff = 0.05;

if measure == 1
    if fallet == "1.f"
        riktigtX = 0.43;
        riktigtY = 0.61;
        T = readtable('measure1\fall1_1.csv');
        %T = readtable('fall1_2.csv');
    elseif fallet == "1.b"
        riktigtX = 0.49;
        riktigtY = 1.73;
        T = readtable('measure1\fall1_bak1.csv');
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
    measurementTime = 15;
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

Fs = length(leftAng)/(measurementTime);
leftAngEf = lowpass(leftAng,cutoff,Fs);
rightAngEf = lowpass(rightAng,cutoff,Fs);
%leftAngEf = lowpass(meanLeftAng,cutoff,Fs);
%rightAngEf = lowpass(meanRightAng,cutoff,Fs);
positionEf = [];
nrSkitEf = 0;
for i = 1:length(leftAngEf)
    [curPosEf, ~, Aef, Bef, Cef ] = triangulering(leftAngEf(i),rightAngEf(i),len);
    
    if isnan(curPosEf)
        nrSkitEf = nrSkitEf + 1;
        continue
    end
    
    positionEf = [positionEf;curPosEf];
    allaAef = [allaAef; Aef];
    allaBef = [allaBef; Bef];
    allaCef = [allaCef; Cef];
end
tEf = linspace(0,measurementTime,length(positionEf));

figure(1) %Positiondata
hold on
plot(riktigtX,riktigtY,'gx')
plot([0,len],[0,0],'o')
plot(position(:,1),position(:,2),'*')
plot(mean(position(:,1)),mean(position(:,2)),'co')
plot(positionEf(:,1),positionEf(:,2),'*')
plot(mean(positionEf(:,1)),mean(positionEf(:,2)),'o')
legend('Riktiga v�rdet','Beacons','Data','Data medelv�rde','Data efter filter','Data efter filter medelv�rde')

figure(2) %X position �ver tid
hold on
plot(t,position(:,1));
%plot([0,measurementTime],[mean(position(:,1)),mean(position(:,1))]);
plot([0,measurementTime],[riktigtX,riktigtX])
plot(tEf,positionEf(:,1));
%plot([0,measurementTime],[mean(positionEf(:,1)),mean(positionEf(:,1))]);
%plot(t,meanPos(:,1),'k');
legend('X position �ver tid','referensv�rde','X position efter filter')
%legend('X position �ver tid','medelv�rde','riktiga v�rdet','X position efter filter','medelv�rde efter filter','movmean')

figure(3) %Y position �ver tid
hold on
plot(t,position(:,2));
%plot([0,measurementTime],[mean(position(:,2)),mean(position(:,2))]);
plot([0,measurementTime],[riktigtY,riktigtY])
plot(tEf,positionEf(:,2));
%plot([0,measurementTime],[mean(positionEf(:,2)),mean(positionEf(:,2))]);
%plot(t,meanPos(:,2),'k');
legend('Y position �ver tid','referensv�rde','Y position efter filter')
%legend('Y position �ver tid','medelv�rde','riktiga v�rdet','Y position efter filter','medelv�rde efter filter','movmean')

figure(4) %V�nstra vinkel �ver tid
hold on
plot(t,allaA(1:end));
%plot([0,measurementTime],[mean(allaA(1:end)),mean(allaA(1:end))]);
plot([0,measurementTime],[riktigtA,riktigtA]);
plot(tEf,allaAef(1:end));
legend('v�nstra vinkel �ver tid','referensv�rde','vinkel efter filter')
%legend('v�nstra vinkel �ver tid','medelv�rde','riktig vinkel','vinkel efter filter')

figure(5) %H�gra vinkel �ver tid
hold on
plot(t,allaB(1:end));
%plot([0,measurementTime],[mean(allaB(1:end)),mean(allaB(1:end))]);
plot([0,measurementTime],[riktigtB,riktigtB]);
plot(tEf,allaBef(1:end));
legend('h�gra vinkel �ver tid','referensv�rde','vinkel efter filter')
%legend('h�gra vinkel �ver tid','medelv�rde','riktig vinkel','vinkel efter filter')

figure(6)
plot(tEf,meanLeftAng,t,leftAng)
legend('v�ntra mean vinkel', 'v�nstra vinkel')

figure(7)
plot(tEf,meanRightAng,t,rightAng)
legend('h�gra mean vinkel', 'h�gra vinkel')

pause;
close all;