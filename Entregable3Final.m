%% Inicialización (Choque)
% Parámetros del sistema
G = 1;   % Constante de gravitación universal
dt = 0.001;        % Paso de tiempo
TN = 19;           % Tiempo final de simulación

% Masas 
M1 = 1.5;         
M2 = 2.0;         
M3 = 1.0;          

% Posiciones y velocidades iniciales
r1_0 = [5.2, 0.0, 1.3];     
v1_0 = [0.1, 0.2, 0.0];     

r2_0 = [0.0, 0.0, 3.5];     
v2_0 = [0.4, 0.0, 0.2];     

r3_0 = [4.1, 0.0, 0.0];     
v3_0 = [0.0, 0.3, 0.4];   
%% Inicialización (órbita retrógada)

% Parámetros del sistema 
G = 39.478;   % Constante de gravitación universal 
dt = 0.001;        % Paso de tiempo 
TN = 30;           % Tiempo final de simulación

% Masas 
M1 = 1; % Masa del Sol
M2 = 0.0009543; % Masa de Júpiter
M3 = 1e-12; % Masa del Asteroide

% Posiciones y velocidades iniciales 
r1_0 = [0, 0, 0]; v1_0 = [0, 0,0.0];

r2_0 = [0.0, 5.2, 0.0]; v2_0 = [2*pi*5.2/11.86,0.0, 0.0];

r3_0 = [0.0, 5.3, 0]; v3_0 = [(2*pi*5.2/11.86)*0.8125,0, 0];
%% Inicialización (atrapamiento)

% Parámetros del sistema 
G = 39.478;   % Constante de gravitación universal 
dt = 0.001;        % Paso de tiempo 
TN = 10;           % Tiempo final de simulación

% Masas 
M1 = 1; 
M2 = 0.0009543;
M3 = 1e-12;

% Posiciones y velocidades iniciales 
r1_0 = [0, 0, 0]; v1_0 = [0, 0,0.0];

r2_0 = [5.2, 0.0, 0.0]; v2_0 = [0.0, 2*pi*5.2/11.86, 0.0];

r3_0 = [5.2, 0.0, 0.5]; v3_0 = [0.0, (2*pi*5.2/11.86)*0.99,0.1];

%% Inicialización de variables 
N = ceil(TN/dt);     % Número de pasos de tiempo 
t=linspace(0,TN,N);

%Posiciones 
r1 = zeros(N, 3); 
r2 = zeros(N, 3); 
r3 = zeros(N, 3);

% Velocidades
v1 = zeros(N, 3); 
v2 = zeros(N, 3); 
v3 = zeros(N, 3);

% Velocidades para medio paso 
v1_half = zeros(N, 3); v2_half = zeros(N,3); v3_half = zeros(N, 3);

% Asignar condiciones iniciales 
r1(1, :) = r1_0; 
r2(1, :) = r2_0; 
r3(1, :) = r3_0;

v1(1, :) = v1_0; 
v2(1, :) = v2_0; 
v3(1, :) = v3_0;

%% Bucle principal de simulación
for i = 1:N-1
    % Cálculo de las fuerzas
    F1 = calculateForce(r1(i, :), r2(i, :), M1, M2, G) + calculateForce(r1(i, :), r3(i, :), M1, M3, G);
    F2 = calculateForce(r2(i, :), r1(i, :), M2, M1, G) + calculateForce(r2(i, :), r3(i, :), M2, M3, G);
    F3 = calculateForce(r3(i, :), r1(i, :), M3, M1, G) + calculateForce(r3(i, :), r2(i, :), M3, M2, G);
      
    % Velocidad (half step)
    v1_half(i,:) = v1(i,:) + 0.5 * F1/M1 * dt;
    v2_half(i,:) = v2(i,:) + 0.5 * F2/M2 * dt;
    v3_half(i,:) = v3(i,:) + 0.5 * F3/M3 * dt;

    % Actualización de las posiciones
    r1(i+1, :) = r1(i, :) + v1_half(i, :) * dt;
    r2(i+1, :) = r2(i, :) + v2_half(i, :) * dt;
    r3(i+1, :) = r3(i, :) + v3_half(i, :) * dt;
    
    % Cálculo de las nuevas fuerzas
    F1_new = calculateForce(r1(i+1, :), r2(i+1, :), M1, M2, G) + calculateForce(r1(i+1, :), r3(i+1, :), M1, M3, G);
    F2_new = calculateForce(r2(i+1, :), r1(i+1, :), M2, M1, G) + calculateForce(r2(i+1, :), r3(i+1, :), M2, M3, G);
    F3_new = calculateForce(r3(i+1, :), r1(i+1, :), M3, M1, G) + calculateForce(r3(i+1, :), r2(i+1, :), M3, M2, G);
    
    % Cálculo de las velocidades
    v1(i+1, :) = v1_half(i, :) + 0.5 * F1_new/M1 * dt;
    v2(i+1, :) = v2_half(i, :) + 0.5 * F2_new/M2 * dt;
    v3(i+1, :) = v3_half(i, :) + 0.5 * F3_new/M3 * dt;
end
%% Criterio de choque
threshold = 0.01; % Valor máximo de acercamiento para definir un choque
collision = false;
collision_position = zeros(1, 3); % Almacena la posición de la colisión

for i = 1:N
    if norm(r1(i, :) - r2(i, :)) <= threshold
        collision = true;
        collision_position = (r1(i, :) + r2(i, :))/2; % Calcula el promedio de las posiciones de los cuerpos en el momento de la colisión
        collision_index=i;
        break;
    end
    if norm(r1(i, :) - r3(i, :)) <= threshold
        collision = true;
        collision_position = (r1(i, :) + r3(i, :))/2; % Calcula el promedio de las posiciones de los cuerpos en el momento de la colisión
        collision_index=i;
        break;
    end
    if norm(r2(i, :) - r3(i, :)) <= threshold
        collision = true;
        collision_position = (r2(i, :) + r3(i, :))/2; % Calcula el promedio de las posiciones de los cuerpos en el momento de la colisión
        collision_index=i;
        break;
    end
end

if collision
    disp('Se ha producido un choque entre dos cuerpos.');
    disp(['Posición de la colisión: (', num2str(collision_position(1)), ', ', num2str(collision_position(2)), ', ', num2str(collision_position(3)), ')']);
    % Elimina los datos después de la colisión
    r1(collision_index:end,:)=[];
    r2(collision_index:end,:)=[];
    r3(collision_index:end,:)=[];
    
    v1(collision_index:end,:)=[];
    v2(collision_index:end,:)=[];
    v3(collision_index:end,:)=[];

    t(collision_index:end)=[];
    N=collision_index;
else
    disp('No se ha producido ningún choque.');
end
%% Cálculo del centro de masas
r_cm = (M1*r1 + M2*r2 + M3*r3)/(M1 + M2 + M3);
v_cm = zeros(N-1,3);
for i=2:N-2
    v_cm(i,1)=(r_cm(i+1,1)-r_cm(i-1,1))/(2*dt);
    v_cm(i,2)=(r_cm(i+1,2)-r_cm(i-1,2))/(2*dt);
    v_cm(i,3)=(r_cm(i+1,3)-r_cm(i-1,3))/(2*dt);
end
v_cm_magnitude = sqrt(sum(v_cm.^2,2));
figure(3)
plot(t(2:end-1), v_cm_magnitude(2:end-1))
title('Velocidad del centro de masa vs tiempo')
ylim ([v_cm(1)-1 v_cm(1)+1])
xlabel('Tiempo')
ylabel('Velocidad del centro de masa')
grid on
%% Cálculo de la energía mecánica
K1 = 0.5*M1*sum(v1.^2, 2);
K2 = 0.5*M2*sum(v2.^2, 2);
K3 = 0.5*M3*sum(v3.^2, 2);
U = -G*M1*M2./sqrt(sum((r1 - r2).^2, 2)) - G*M1*M3./sqrt(sum((r1 - r3).^2, 2)) - G*M2*M3./sqrt(sum((r2 - r3).^2, 2));
K = K1 + K2 + K3;
E = K + U;
figure(4);
plot(t,K,t,U,t,E);
title('Energía en función del tiempo');
xlabel('Tiempo');
ylabel('Energía');
legend('Cinética','Potencial','Mecánica','Location','best')
grid on;
%% Gráfico de las trayectorias y animación (Tres cuerpos cualquiera)
figure(2);
h(1)=plot3(r1(:, 1), r1(:, 2), r1(:, 3), 'Color', "#77AC30",LineWidth=1.5);
hold on;
h(2)=plot3(r2(:, 1), r2(:, 2), r2(:, 3), 'Color', "#A2142F",LineWidth=1.5);
h(3)=plot3(r3(:, 1), r3(:, 2), r3(:, 3), 'Color', "#0072BD",LineWidth=1.5);
h(4)=plot3(r_cm(:,1),r_cm(:,2),r_cm(:,3),LineWidth=1.5);
h(5)=plot3(r1(1,1),r1(1,2),r1(1,3),'r*');
h(6)=plot3(r2(1,1),r2(1,2),r2(1,3),'r*');
h(7)=plot3(r3(1,1),r3(1,2),r3(1,3),'r*');
title('Trayectorias de los cuerpos');
xlabel('X');
ylabel('Y');
zlabel('Z');
if norm(collision_position)>0
    h(8)=scatter3(collision_position(1),collision_position(2),collision_position(3),'filled','r');
    legend(h([1,2,3,4,5,8]),{'Cuerpo 1', 'Cuerpo 2', 'Cuerpo 3','Centro de masa','Posición Inicial','Colisión'});
else
    legend(h([1,2,3,4,5]),{'Cuerpo 1', 'Cuerpo 2', 'Cuerpo 3','Centro de masa','Posición Inicial'})
end
grid on;

%Animación de las órbitas
figure(1);
h=plot3(r1(:, 1), r1(:, 2), r1(:, 3),'Color', "#77AC30",LineWidth=1.5);
hold on;
h2=plot3(r2(:, 1), r2(:, 2), r2(:, 3), 'Color', "#A2142F",LineWidth=1.5);
h3=plot3(r3(:, 1), r3(:, 2), r3(:, 3), 'Color', "#0072BD",LineWidth=1.5);
h4=plot3(r_cm(:,1),r_cm(:,2),r_cm(:,3),LineWidth=1.5);
title('Órbitas de los cuerpos');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Cuerpo 1', 'Cuerpo 2', 'Cuerpo 3','Centro de masa');
grid on;
axis equal;

skip = 150; % Saltar 'skip' pasos de tiempo en cada iteración de la animación

for i = 1:skip:N-1
    set(h, 'XData', r1(1:i, 1), 'YData', r1(1:i, 2), 'ZData', r1(1:i, 3));
    set(h2, 'XData', r2(1:i, 1), 'YData', r2(1:i, 2), 'ZData', r2(1:i, 3));
    set(h3, 'XData', r3(1:i, 1), 'YData', r3(1:i, 2), 'ZData', r3(1:i, 3));
    set(h4, 'XData', r_cm(1:i, 1), 'YData', r_cm(1:i, 2), 'ZData', r_cm(1:i, 3));
    drawnow;
end
%% Gráfico de las trayectorias y animación (Sol, Jupiter y Asteroide)
figure(2);
h(1)=scatter3(r1(:, 1), r1(:, 2), r1(:, 3), 'MarkerFaceColor', "#EDB120", 'MarkerEdgeColor', "#D95319");
hold on;
h(2)=plot3(r2(:, 1), r2(:, 2), r2(:, 3), 'Color', "#A2142F");
h(3)=plot3(r3(:, 1), r3(:, 2), r3(:, 3), 'Color', "#0072BD");
% h(4)=plot3(r_cm(:,1),r_cm(:,2),r_cm(:,3));
% h(5)=plot3(r1(1,1),r1(1,2),r1(1,3),'r*');
h(4)=plot3(r2(1,1),r2(1,2),r2(1,3),'r*');
h(5)=plot3(r3(1,1),r3(1,2),r3(1,3),'r*');
title('Trayectorias de los cuerpos');
xlabel('X (UA)');
ylabel('Y (UA)');
zlabel('Z (UA)');
if norm(collision_position)>0
    h(6)=scatter3(collision_position(1),collision_position(2),collision_position(3),'filled','r');
    legend(h([1,2,3,4,6]),{'Sol', 'Júpiter', 'Asteroide','Posición Inicial','Colisión'});
else
    legend(h([1,2,3,4]),{'Sol', 'Júpiter', 'Asteroide','Posición Inicial'})
end
grid on;

% Animación de las órbitas
figure(1);
h=scatter3(r1(:, 1), r1(:, 2), r1(:, 3), 'MarkerFaceColor', "#EDB120", 'MarkerEdgeColor', "#D95319");
hold on;
h2=plot3(r2(:, 1), r2(:, 2), r2(:, 3), 'Color', "#A2142F");
h3=plot3(r3(:, 1), r3(:, 2), r3(:, 3), 'Color', "#0072BD");
title('Órbitas de los cuerpos');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Sol', 'Júpiter', 'Asteroide');
grid on;
axis equal;

skip = 150; % Saltar 'skip' pasos de tiempo en cada iteración de la animación

for i = 1:skip:N-1
    set(h, 'XData', r1(1:i, 1), 'YData', r1(1:i, 2), 'ZData', r1(1:i, 3));
    set(h2, 'XData', r2(1:i, 1), 'YData', r2(1:i, 2), 'ZData', r2(1:i, 3));
    set(h3, 'XData', r3(1:i, 1), 'YData', r3(1:i, 2), 'ZData', r3(1:i, 3));
    drawnow;
end

% Función para calcular la fuerza gravitacional entre dos cuerpos
function F = calculateForce(r1, r2, M1, M2, G)
    r = r2 - r1;
    dist = norm(r);
    F = G*M1*M2/(dist^3) * r;
end
