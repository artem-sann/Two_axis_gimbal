duration = 5; % секунды
fs = 1000;     % частота выборки (Гц)
N = duration * fs;

data = zeros(N, 2);
disp("Сбор данных...");

for i = 1:N
    line = readline(s);
    nums = sscanf(line, '%f,%f');
    if numel(nums) == 2
        data(i, :) = nums';
    end
end

disp("Готово. Построение графика...");
t = linspace(0, duration, N);

figure;
plot(t, data(:,1), 'b', 'DisplayName', 'Сгенерированный sin');
hold on;
plot(t, data(:,2), 'r', 'DisplayName', 'Измеренный угол');
xlabel('Время (с)');
ylabel('Значения переменных');
legend;
title('График заданного угла и измеренного');
grid on;