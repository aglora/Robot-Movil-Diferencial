% Función para mostrar telemetría de robot móvil (VÁLIDO para MODO 7) 
% Autores: Álvaro García Lora, Sergio León Doncel, Isaac Rodríguez García

function telemetriaM7(archivo)
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

% tel(:,2); %Velocidad A
% tel(:,3); %Velocidad B
% tel(:,4); %RefVelocidad A
% tel(:,5); %RefVelocidad B
% tel(:,6); %Pose X
% tel(:,7); %Pose Y
% tel(:,8); %Pose phi
% tel(:,9); %PWM A
% tel(:,10); %PWM B

figure(1) %VELOCIDADES 
subplot(3,1,1);
hold on
plot(tiempo,tel(:,4),'k','LineWidth',2); %RefVelocidad A
plot(tiempo,tel(:,2),'r','LineWidth',2); %Velocidad A
xlabel('Tiempo (s)');
ylabel('Velocidad (rpm)');
title('Velocidades ruedas');
legend('Ref.Vel.der','Vel.der');
grid minor;
grid on;
hold off

subplot(3,1,2);
hold on
plot(tiempo,tel(:,5),'k','LineWidth',2); %RefVelocidad B
plot(tiempo,tel(:,3),'b','LineWidth',2); %Velocidad B
xlabel('Tiempo (s)');
ylabel('Velocidad (rpm)');
legend('Ref.Vel.izq','Vel.izq');
grid minor;
grid on;
hold off

subplot(3,1,3);
hold on
plot(tiempo,tel(:,9),'r','LineWidth',2); %PWM A
plot(tiempo,tel(:,10),'b','LineWidth',2); %PWM B
xlabel('Tiempo (s)');
ylabel('PWM');
title('PWM aplicada');
legend('PWM.der','PWM.izq');
grid minor;
grid on;
hold off

figure(2) %POSE ROBOT
subplot(3,1,1);
hold on
plot(tiempo,tel(:,6),'r','LineWidth',2); %Pose X
xlabel('Tiempo (s)');
ylabel('Distancia (m)');
title('Pose robot');
legend('X');
grid minor;
grid on;
hold off

subplot(3,1,2);
hold on
plot(tiempo,tel(:,7),'b','LineWidth',2); %Pose Y
xlabel('Tiempo (s)');
ylabel('Distancia (m)');
legend('Y');
grid minor;
grid on;
hold off

subplot(3,1,3);
hold on
plot(tiempo,tel(:,8),'g','LineWidth',2); %Pose phi
xlabel('Tiempo (s)');
ylabel('Ángulo (º)');
legend('phi');
grid minor;
grid on;
hold off

figure(3) %TRAYECTORIA
hold on
plot([0 1],[0 0],'k','LineWidth',5); %Trayectoria ideal
plot([1 1],[0 1],'k','LineWidth',5); %Trayectoria ideal
plot([1 0],[1 1],'k','LineWidth',5); %Trayectoria ideal
plot([0 0],[1 0],'k','LineWidth',5); %Trayectoria ideal
plot(tel(:,6),tel(:,7),'g','LineWidth',2); %Trayectoria seguida
plot([0],[0],'rx','LineWidth',5,'MarkerSize',30); %Punto inicial
xlabel('X(m)');
ylabel('Y(m)');
title('Trayectoria plana')
grid minor;
grid on;
hold off

end