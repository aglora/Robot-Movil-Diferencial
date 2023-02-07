% Función para mostrar telemetría de robot móvil (VÁLIDO para MODOS 1 al 6) 
% Autores: Álvaro García Lora, Sergio León Doncel, Isaac Rodríguez García

function telemetriaM1_M6(archivo)
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

modo = tel(1,5);
tiempo = tiempo/1000; %Tiempo en segundos

switch(modo)
    case 1
        disp('1')
        figure(1) %BASICO
        subplot(2,1,1);
        hold on
        plot(tiempo,tel(:,4),'g','LineWidth',3);
        plot(tiempo,tel(:,2),'b','LineWidth',2.0);
        plot(tiempo,tel(:,3),'r','LineWidth',1.0); 
        xlabel('Tiempo (s)');
        ylabel('Distancia (cm)');
        title('Sensores');
        legend('Referencia','Dist. izq', 'Dist. der');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        plot(tiempo,tel(:,6),'r','LineWidth',2.5);
        plot(tiempo,tel(:,7),'b','LineWidth',1.0);
        title('Actuadores');
        xlabel('Tiempo (s)');
        ylabel('PWM');
        legend('PWM dcha.', 'PWM izq.');
        grid on;
        grid minor;
        hold off

        figure(2) %ERRORES
        hold on
        error=tel(:,4)-tel(:,2);
        plot(tiempo,error,'b','LineWidth',2.0);
        plot([0 12],[0 0],'r');
        xlabel('Tiempo (s)');
        ylabel('Error (cm)');
        title('Evolución temporal del error');
        grid minor;
        grid on;
        hold off
    case 2
        disp('2')
        figure(1) %BASICO
        subplot(2,1,1);
        hold on
        plot(tiempo,tel(:,4),'g','LineWidth',3);
        plot(tiempo,tel(:,2),'b','LineWidth',2);
        plot(tiempo,tel(:,3),'r','LineWidth',1); 
        xlabel('Tiempo (s)');
        ylabel('Distancia (cm)');
        title('Sensores');
        legend('Referencia','Dist. izq', 'Dist. der');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        plot(tiempo,tel(:,6),'r','LineWidth',1.5);
        plot(tiempo,tel(:,7),'b','LineWidth',1.5);
        title('Actuadores');
        xlabel('Tiempo (s)');
        ylabel('PWM');
        legend('PWM dcha.', 'PWM izq.');
        grid on;
        grid minor;
        hold off
        
        
        figure(2) %ERRORES
        subplot(2,1,1);
        hold on
        error1=tel(:,4)-tel(:,3);
        plot(tiempo,error1,'r','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error d1 (cm)');
        title('Evolución temporal del error');
        legend('Error d1');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        error2=tel(:,4)-tel(:,2);
        plot(tiempo,error2,'b','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error d2 (cm)');
        legend('Error d2');
        grid minor;
        grid on;
        hold off

    case 3

        figure(1) %BASICO
        disp('3')
        subplot(2,1,1);
        hold on
        plot(tiempo,tel(:,4),'g','LineWidth',3);
        plot(tiempo,tel(:,2),'b','LineWidth',1.5);
        plot(tiempo,tel(:,3),'r','LineWidth',1.5); 
        xlabel('Tiempo (s)');
        ylabel('Distancia (cm)');
        title('Sensores');
        legend('Distancia Inicial','Dist. izq', 'Dist. der');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        plot(tiempo,tel(:,6),'r','LineWidth',1.5);
        plot(tiempo,tel(:,7),'b','LineWidth',1.5);
        title('Actuadores');
        xlabel('Tiempo (s)');
        ylabel('PWM');
        legend('PWM dcha.', 'PWM izq.');
        grid on;
        grid minor;
        hold off

        figure(2) %ERRORES
        subplot(2,1,1);
        hold on
        error1=tel(:,4)-tel(:,3);
        plot(tiempo,error1,'r','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error d1 (cm)');
        title('Evolución temporal del error');
        legend('Error d1');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        error2=tel(:,4)-tel(:,2);
        plot(tiempo,error2,'b','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error d2 (cm)');
        legend('Error d2');
        grid minor;
        grid on;
        hold off
        
    case 4
        disp('4')
        figure(1) %BASICO
        subplot(2,1,1);
        hold on
        plot(tiempo,tel(:,4),'g','LineWidth',2);
        plot(tiempo,tel(:,2),'b','LineWidth',1.5);
        plot(tiempo,tel(:,3),'r','LineWidth',1.5); 
        xlabel('Tiempo (s)');
        ylabel('Distancia (cm)');
        title('Sensores');
        legend('Referencia','Dist. izq', 'Dist. der');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        plot(tiempo,tel(:,6),'r','LineWidth',1.5);
        plot(tiempo,tel(:,7),'b','LineWidth',1.5);
        title('Actuadores');
        ylabel('PWM');
        xlabel('Tiempo (s)');
        legend('PWM dcha.', 'PWM izq.');
        grid on;
        grid minor;
        hold off
        
        figure(2) %ERRORES
        subplot(2,1,1);
        hold on
        error1=tel(:,4)-tel(:,3);
        plot(tiempo,error1,'r','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error d1 (cm)');
        title('Evolución temporal del error');
        legend('Error d1');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        error2=tel(:,4)-tel(:,2);
        plot(tiempo,error2,'b','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error d2 (cm)');
        legend('Error d2');
        grid minor;
        grid on;
        hold off

        figure(3) %ERRORES
        hold on
        error=tel(:,2)-tel(:,3);
        plot(tiempo,error,'g','LineWidth',2.0);
        plot([0 30],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('d1-d2 (cm)');
        title('Diferencia de distancias');
        grid minor;
        grid on;
        hold off

    case 5
        disp('5')
        figure(1) %BASICO
        subplot(2,1,1);
        hold on
        plot(tiempo,tel(:,4),'g','LineWidth',2);
        plot(tiempo,tel(:,2),'b','LineWidth',1.5);
        plot(tiempo,tel(:,3),'r','LineWidth',1.5); 
        xlabel('Tiempo (s)');
        ylabel('Velocidad (rpm)');
        title('Sensores');
        legend('Referencia','Vel. izq', 'Vel. der');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        plot(tiempo,tel(:,6),'r','LineWidth',1.5);
        plot(tiempo,tel(:,7),'b','LineWidth',1.5);
        title('Actuadores');
        xlabel('Tiempo (s)');
        ylabel('PWM');
        legend('PWM dcha.', 'PWM izq.');
        grid on;
        grid minor;
        hold off

        figure(2) %ERRORES
        subplot(2,1,1);
        hold on
        error1=tel(:,4)-tel(:,3);
        plot(tiempo,error1,'r','LineWidth',2.0);
        plot([0 35],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error vel.der (rpm)');
        title('Evolución temporal del error');
        grid minor;
        grid on;
        hold off
        
        subplot(2,1,2);
        hold on
        error2=tel(:,4)-tel(:,2);
        plot(tiempo,error2,'b','LineWidth',2.0);
        plot([0 35],[0 0],'k');
        xlabel('Tiempo (s)');
        ylabel('Error vel.izq (rpm)');
        grid minor;
        grid on;
        hold off

    case 6
        figure(1) %BASICO
        disp('6')
        subplot(2,1,1);
        hold on
        plot([0 22],[40 40],'k','LineWidth',2);
        plot(tiempo,tel(:,2),'b','LineWidth',1.5);
        plot(tiempo,tel(:,3),'r','LineWidth',1.5); 
        xlabel('Tiempo (s)');
        ylabel('Velocidad (rpm)');
        title('Sensores');
        legend('Referencia Vel.Cte','Vel. izq', 'Vel. der');
        grid on;
        grid minor;
        hold off
        
        subplot(2,1,2);
        hold on
        plot(tiempo,tel(:,6),'r','LineWidth',1.5);
        plot(tiempo,tel(:,7),'b','LineWidth',1.5);
        title('Actuadores');
        xlabel('Tiempo (s)');
        ylabel('PWM');
        legend('PWM dcha.', 'PWM izq.');
        grid on;
        grid minor;
        hold off

        figure(2)
        hold on
        plot(tiempo,tel(:,4),'g','LineWidth',2);
        plot(tiempo,tel(:,3),'r','LineWidth',1.5); 
        xlabel('Tiempo (s)');
        ylabel('Velocidad (rpm)');
        title('Sensores');
        legend('Referencia Vel.der', 'Vel. der');
        grid on;
        grid minor;
        hold off
        
end
end