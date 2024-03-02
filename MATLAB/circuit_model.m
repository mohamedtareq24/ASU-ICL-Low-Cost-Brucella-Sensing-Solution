clc;
clear;
close all;
%f = [0.1,109, 116, 126, 135, 148, 160, 173, 187, 202, 219,236, 256, 276, 299, 323, 349, 377, 408, 441, 477, 516, 557, 603, 651, 704, 761, 823, 890, 962, 1000, 1125, 1216, 1315, 1421, 1537, 1661, 1796, 1942, 2100, 2270, 2454, 2653, 2869, 3102, 3353, 3626, 3920, 4238, 4582, 4954, 5356, 5791, 6261, 6769, 7319, 7913, 8555, 9250, 10000 , 50000];
f   = [100	281.8	350	466	633	786	1.05E+03	1.50E+03	2.00E+03	2.40E+03	3.28E+03	4.16E+03	4.60E+03	5.20E+03	5.90E+03	6.30E+03	7.16E+03	7.60E+03	8.03E+03	8.39E+03	8.90E+03	9.30E+03	1.07E+04];
N = 1 ;
Ru  = 10e3 / N; 
Cdl = 1e-9 * N;
Rp  = 340e3 / N;
f_index  = 1;
% Define the noise level (adjust as needed)
noise_level = 0.00;  % Example noise level
t = linspace(0,2/f(f_index),1001); % time span of  2 cycles

Vsignal   = zeros(length(f), length(t));
Isignal   = zeros(length(f), length(t));  
sumSignal = zeros(length(f), length(t));
w         = 2*pi.*f;
ref = [244	244	244	244	244	250	248	248	254	256	256	256	256	256	256	256	256	252	252	252	252	252	252]
for i = 1:length(f)
    noise = noise_level * randn(size(Vsignal(i,:)));
    Randells      (i)       = Ru + Rp / (Rp * 1i * Cdl .* w(i) + 1);                  %%  resistance 
    Vsignal       (i,:)     = 1.65  + 0.244*sin(2*pi*f(i).*t);                          %%  ESP32 signal 
    Isignal       (i,:)     = 2  + 100000 *(-1 .*Vsignal(i,:) / Randells(i)) + noise;       %%  TIA gain     
    sumSignal     (i,:)     = 0.5*(Isignal(i,:) + Vsignal(i,:));                        %% summing amplifer
end




% Create arrays to store the noisy signals
Vsignal_noisy   = zeros(size(Vsignal));
Isignal_noisy   = zeros(size(Isignal));
sumSignal_noisy = zeros(size(sumSignal));
Vsignal_DC      = zeros(length(f), length(t));
Isignal_DC      = zeros(length(f), length(t));
sumSignal_DC    = zeros(length(f), length(t));

for i = 1:length(f)
    
    % Calculate the mean of Vsignal(i,:) only once
    Vsignal_mean = mean(Vsignal(i,:));
    
    Vsignal_DC(i,:) = Vsignal(i,:) - Vsignal_mean;
    
    Vsignal_noisy(i,:) = Vsignal_DC(i,:) + noise;
    
    % Calculate the mean of Isignal(i,:) only once
    Isignal_mean = mean(Isignal(i,:));
    
    Isignal_DC(i,:) = Isignal(i,:) - Isignal_mean;
    
    Isignal_noisy(i,:) = Isignal_DC(i,:) + noise;
    
    % Calculate the mean of sumSignal(i,:) only once
    sumSignal_mean = mean(sumSignal(i,:));
    
    sumSignal_DC(i,:) = sumSignal(i,:) - sumSignal_mean;
    
    sumSignal_noisy(i,:) = sumSignal_DC(i,:) + noise;
    
    
    % Calculate impedance magnitude
    a(i) = max(abs(Vsignal_DC(i, :)));  % Magnitude of voltage signal
    b(i) = max(abs(Isignal_noisy(i, :))) ;  % Magnitude of current signal
    c(i) = 2 * max(abs(sumSignal_DC(i, :)));  % Magnitude of sum signal divided by the gain
    
    Z(i) = a(i) / b(i) * 100000;  % Impedance magnitude
    
    % Calculate phase angle phi (in degrees)
    cosC = ((a(i)^2) + (b(i)^2) - (c(i)^2)) / (2 * a(i) * b(i));  % Cosine rule
    phi(i) =  acos(cosC) * 180 / pi;
end

    real_part_Z = zeros(1, length(f));
    imaginary_part_Z = zeros(1, length(f));

for i = 1:length(f)
    % Calculate the real and imaginary parts of Z using magnitude (a(i)) and phase (phi(i))
    real_part_Z(i) = Z(i) * cosd(phi(i));  % Real part
    imaginary_part_Z(i) = Z(i) * sind(phi(i));  % Imaginary part
end


% Plot the Isignal for a specific frequency (e.g., f_index)

% Create a figure for DC signals

figure;

% Plot Vsignal_DC and label it with "DC V"
plot(t, Vsignal_DC(f_index, :), 'DisplayName', 'DC V');
hold on;

% Plot Isignal_DC and label it with "DC I"
plot(t, Isignal_DC(f_index, :), 'DisplayName', 'DC I');

% Plot sumSignal_DC and label it with "DC Sum"
plot(t, sumSignal_DC(f_index, :), 'DisplayName', 'DC Sum');

xlabel('Time');
ylabel('Amplitude');
title('DC Signals');
legend('Location', 'Best');
hold off;

% Create a figure for Noisy signals
figure;

% Plot Vsignal_noisy and label it with "Noisy V"
plot(t, Vsignal_noisy(f_index, :), 'DisplayName', 'Noisy V');
hold on;

% Plot Isignal_noisy and label it with "Noisy I"
plot(t, Isignal_noisy(f_index, :), 'DisplayName', 'Noisy I');

% Plot sumSignal_noisy and label it with "Noisy Sum"
plot(t, sumSignal_noisy(f_index, :), 'DisplayName', 'Noisy Sum');

xlabel('Time');
ylabel('Amplitude');
title('Noisy Signals');
legend('Location', 'Best');
hold off;

% Create a figure for No Edit signals
figure;

% Plot Vsignal and label it with "V"
plot(t, Vsignal(f_index, :), 'DisplayName', 'V');
hold on;

% Plot Isignal and label it with "I"
plot(t, Isignal(f_index, :), 'DisplayName', 'I');

% Plot sumSignal and label it with "Sum"
plot(t, sumSignal(f_index, :), 'DisplayName', 'Sum');

xlabel('Time');
ylabel('Amplitude');
title('raw');
legend;
hold off;


figure;
scatter (real_part_Z(f_index:end),imaginary_part_Z(f_index:end),'*');
%scatter (real(Randells),imag(Randells),'*');
xlabel('Real','FontSize', 15);
ylabel('- Imaginary','FontSize', 15);
axis equal
legend ("simulation")
%set(gca, 'YDir','reverse')
%set(gca,'xtick',[0:1000:1e6])
grid on
hold on 

hold off

%% tsting
%r = real(Randells);
%j = imag(Randells); 
%i = find (-1.*j == max(-1.*j)); % index
%Z_abs = sqrt(r(i)^2 + j(i)^2); 
%fmax = f (i);
%Cdltst = 1 / (2 * pi * fmax * (sqrt(Z_abs^2 + Rp^2) - Ru))
%Cdldif = Cdltst - Cdl

