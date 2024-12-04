clc
clear all
close all

t0 = 0; W0 = 0;
W_max = 5000 % Максимальная скорость оси
t_max = 0.25; % Максимальное время процесса
t1 = 2*t_max/5;
t2 = 3*t_max/5;
E_max = (W_max - W0)/(t1-t0) % Максимальное ускорение оси

t = [t0   t1  t2  t_max ];
W = [W0  W_max  W_max  W0 ]; 
E = [E_max E_max 0 0 -E_max -E_max];
t_e = [t0   t1 t1  t2 t2 t_max ];

S = (t_max+(t_max/3))*W_max/2 %пройденный путь в градусах, по тз = 720

%---------------График скоростей---------------------
figure;
plot(t, W, 'LineWidth',2, 'color', 'blue')
xlim([0 t_max*1.5]);
ylim([0 W_max+W_max/3]);
title('Скорость осей');
grid on;
xlabel('Время, Сек.') ;
ylabel('Угловая скорсоть, °/сек ');
legend({'ω','Second curve'},'Location','northeast')
%----------------------------------------------------
%---------------График ускорений---------------------
figure;
plot(t_e, E, 'LineWidth',2, 'color', 'red')
xlim([0 t_max*1.5]);
ylim([-(E_max+E_max/3) E_max+E_max/3]);
title('Ускорения осей');
grid on;
xlabel('Время, Сек.') ;
ylabel('Угловое ускорение, °/сек^2 ');
legend({'ε','Second curve'},'Location','northeast')
%----------------------------------------------------

