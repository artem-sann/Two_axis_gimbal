clear all;
close all;
% Настройка порта
port = 'COM10'; % замените на ваш порт
baud = 115200;

s = serialport(port, baud);
configureTerminator(s, "CR/LF"); % CR/LF = \r\n
flush(s);