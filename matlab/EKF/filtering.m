function [filt_signalX, filt_signalY] = filtering(X, Y, Ts)

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
%figure(1)
%freqz(sos, 1024, Fs);                % Plot Filter Response (Bode Plot)

%dt = 0.25;
t = (1:length(X))*Ts;

filt_signalX = filtfilt(sos, g, X);    % Filter Signal
filt_signalY = filtfilt(sos, g, Y);    % Filter Signal


end
