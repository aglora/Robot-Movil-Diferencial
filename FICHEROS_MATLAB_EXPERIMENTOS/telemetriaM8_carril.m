% Función para mostrar telemetría de robot móvil (VÁLIDO para PROYECTO : MODO 8) 
% Autores: Álvaro García Lora, Sergio León Doncel, Isaac Rodríguez García

function telemetriaM8_carril(archivo)
close all;
tel=load(archivo);
muestras=length(tel);
disp('Incremento de tiempo mínimo:'); disp(min(tel(:,1)));
disp('Incremento de tiempo máximo:');disp(max(tel(:,1)));
disp('Incremento de tiempo promedio:'); disp(mean(tel(:,1)));
tiempo=zeros(1,muestras);
tiempo(1)=tel(1,1); %Vector de tiempo absoluto
for i=2:muestras
    tiempo(i)=tiempo(i-1)+tel(i,1);
end

tiempo = tiempo/1000; %Tiempo en segundos

% tel(:,2); %Ref Vel A
% tel(:,3); %Ref Vel B
% tel(:,4); %RefAng
% tel(:,5); %GoalX
% tel(:,6); %GoalY
% tel(:,7); %PoseX
% tel(:,8); %PoseY
% tel(:,9); %PosePhi
% tel(:,10); %estado
% tel(:,11); %distanciaDerecha
% tel(:,12); %distanciaIzquierda
% tel(:,13); %distanciaFrontal

figure(1) %REFERENCIA VELOCIDADES 
hold on
plot(tiempo,tel(:,2),'k','LineWidth',2); %RefVelocidad A
plot(tiempo,tel(:,3),'r','LineWidth',2); %RefVelocidad B
xlabel('Tiempo (s)');
ylabel('Velocidad (rpm)');
title('Referencia Velocidades ruedas');
legend('Ref.Vel.der','Ref.Vel.der');
grid minor;
grid on;
hold off

figure(2) %POSE ROBOT
subplot(3,1,1);
hold on
plot(tiempo,tel(:,7),'r','LineWidth',2); %Pose X
xlabel('Tiempo (s)');
ylabel('Distancia (m)');
title('Pose robot');
legend('poseX');
grid minor;
grid on;
hold off

subplot(3,1,2);
hold on
plot(tiempo,tel(:,8),'b','LineWidth',2); %Pose Y
xlabel('Tiempo (s)');
ylabel('Distancia (m)');
legend('poseY');
grid minor;
grid on;
hold off

subplot(3,1,3);
hold on
plot(tiempo,tel(:,4),'k','LineWidth',2); %RefAng
plot(tiempo,tel(:,9),'g','LineWidth',2); %Pose phi
xlabel('Tiempo (s)');
ylabel('Ángulo (º)');
legend('RefAng','posePhi');
grid minor;
grid on;
hold off

figure(3) %TRAYECTORIA
hold on
plot([0],[0],'rx','LineWidth',5,'MarkerSize',30); %Punto inicial
plot(tel(:,7),tel(:,8),'k','LineWidth',2); %Trayectoria seguida
xlabel('X(m)');
ylabel('Y(m)');
title('Trayectoria plana')
grid minor;
grid on;
hold off

figure(4) %Distancias Ultrasonidos
subplot(3,1,1);
hold on
plot(tiempo,tel(:,13),'k','LineWidth',2); %Distancia Frontal
xlabel('Tiempo (s)');
ylabel('Distancia (cm)');
title('Distancias Medidas Ultrasonidos');
legend('Distancia Frontal');
grid minor;
grid on;
hold off

subplot(3,1,2);
hold on
plot(tiempo,tel(:,11),'r','LineWidth',2); %Distancia Derecha
xlabel('Tiempo (s)');
ylabel('Distancia (cm)');
legend('Distancia Derecha');
grid minor;
grid on;
hold off

subplot(3,1,3);
hold on
plot(tiempo,tel(:,12),'b','LineWidth',2); %Distancia Izquierda
xlabel('Tiempo (s)');
ylabel('Distancia (cm)');
legend('Distancia Izquierda');
grid minor;
grid on;
hold off

disp('PROYECTO')
end