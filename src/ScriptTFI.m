clear all; close all; clc;
pkg load control;

% Parámetros del Sistema
m = 5;
k = 100;
c = 2;

% Cálculo de Zeta, Frec. Natural y Frec. Amortiguada
zeta = c / (2*sqrt(k*m))
omega_n = sqrt(k/m)
omega_d = omega_n * sqrt(1 - zeta^2)

% Períodos asociados
Tn = (2*pi) / omega_n
Td = (2*pi) / omega_d

% Función de Transferencia: Fuerza -> Desplazamiento
G = tf(1, [m, c, k]);

% Polos
figure;
pzmap(G);
title("Mapa de polos y ceros del sistema");
grid on;

% Respuesta a escalón
figure;
step(G);
title("Respuesta a entrada escalón");
xlabel("Tiempo [s]");
ylabel("Desplazamiento [m]");
grid on;

% Respuesta a impulso
figure;
impulse(G);
for i = 1:20
  line ("xdata",[Td*i,Td*i], "ydata",[-1,1], "linewidth", 0.5, "color", "red")
endfor
title("Respuesta a entrada impulso");
xlabel("Tiempo [s]");
ylabel("Desplazamiento [m]");
grid on;

% Calculo de Sobreimpulso (Overshoot)
[y, t] = step(G);
y_max = max(y);
overshoot = (y_max - y(end)) / y(end) * 100;

% Parametros Sensor
Ks = 1;
fc = 100;
tau_s = 1 / (2 * pi * fc)

% Parametros Actuador
Ka = 10;
tau_a = 0.01;

% Planta Extendida
S = tf(Ks, [tau_s, 1]);
A = tf(Ka, [tau_a, 1]);
Gext = A * S * G;

% Polos Planta Extendida
pole(Gext)
figure;
pzmap(Gext);
title("Mapa de polos y ceros de Planta Extendida");
grid on;

% Etapa Polos Deseados
pkg load symbolic;

% --- ZETA DESEADO ---
syms zeta_d Mp;
eq = Mp == exp((-pi*zeta_d)/sqrt(1-zeta_d^2));

% Despeje de zeta_d en funcion de Mp
solucion = solve(eq, zeta_d)

% Solucion
Mp_val = 0.05; % 5%
zeta_sol = -sqrt(1 / ((log(Mp_val))^2 + pi^2)) * log(Mp_val)

% --- OMEGA_N DESEADO ---
syms omega_nd Ts;
eq2 = Ts == 4/(zeta_d * omega_nd)

solucion_2 = solve(eq2, omega_nd)

% Reemplazando con valores numericos
Ts_val = 1.2;
omega_sol = 4/(Ts_val * zeta_sol)

% Calculos Controlador 
% ---- TODO ESTO PERTENECE A PD, que ya no lo vamos a usar, se reemplaza por PID
% beta = tau_a + tau_s;
% K_d = (beta * m * (omega_sol^2) + 2 * m * zeta_sol * omega_sol - c - k * beta) / (Ka * Ks)
% K_p = (m * (omega_sol^2) - k) / (Ka * Ks)
%
% % --- ULTIMA ETAPA ---
% K_d = 3.1526; % 3.1526
% K_p = 1.6653; % 1.6653

% C = tf([K_d, K_p] , 1)

% lazo_abierto_SC = A * G;
% lazo_abierto_CC = C * A * G;
%
% lazo_cerrado_SC = feedback(lazo_abierto_SC, S);
% lazo_cerrado_CC = feedback(lazo_abierto_CC, S);
% [x2, y2] = step(lazo_cerrado_CC);
% y_max_est = x2(end)
% franja_sup = y_max_est*1.02;
% franja_inf = y_max_est*0.98;
%
% figure;
% step(lazo_cerrado_SC, lazo_cerrado_CC, [0:0.001:20]);
% line ("xdata",[Ts_val,Ts_val], "ydata",[-1,1], "linewidth", 0.5, "color", "green");
% line ("xdata",[0,20], "ydata",[franja_sup,franja_sup], "linewidth", 0.5, "color", "green");
% line ("xdata",[0,20], "ydata",[franja_inf,franja_inf], "linewidth", 0.5, "color", "green");
% title("Respuesta a entrada escalón en lazo cerrado");
% xlabel("Tiempo [s]");
% ylabel("Desplazamiento [m]");
% grid on;
%
% figure;
% pzmap(lazo_cerrado_CC);
% title("Mapa de polos y ceros de Planta Extendida");
% grid on;
% ---- TODO ESTO PERTENECE A PD

% --- CONTROLADOR PID (REEMPLAZA AL PD) ---

% Ganancias iniciales (vamos a afinarlas si es necesario)
Kp = 250;
Ki = 180;
Kd = 15;

% PID en forma estándar: C(s) = (Kd*s^2 + Kp*s + Ki)/s
C = tf([Kd Kp Ki], [1 0]);

% Lazo abierto y cerrado
L = C * Gext;
T = feedback(L,1);     % Lazo cerrado completo

% Lazo cerrado sin controlador para comparar
T_sc = feedback(Gext,1);

% --- RESPUESTA AL ESCALON CON PID ---
figure;
step(T, 5);
title("Respuesta en lazo cerrado con PID");
xlabel("Tiempo [s]");
ylabel("Desplazamiento [m]");
grid on;

% --- COMPARACION SIN Y CON CONTROL ---
figure;
step(T_sc, T, 5);
legend("Sin Control", "Con PID");
title("Comparación Lazo Cerrado");
grid on;

% --- ACCION DE CONTROL ---
t = 0:0.001:5;
r = ones(size(t));        % referencia escalón

[y, t] = lsim(T, r, t);   % salida del sistema
u = lsim(C, r - y, t);    % acción de control

figure;
plot(t, u);
title("Acción de Control u(t)");
xlabel("Tiempo [s]");
ylabel("Fuerza del actuador [N]");
grid on;


