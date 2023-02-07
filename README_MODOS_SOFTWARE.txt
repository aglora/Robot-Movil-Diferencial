// NOTAS IMPORTANTES SOBRE LA ARQUITECTURA DE FICHEROS USADA:

- Se ha trabajado sobre PlatformIO, una extensión del IDE Visual Code Studio, la cual nos facilita el desarrollo de código y librerias propias y de terceros. 

- El sistema de archivos:
	
	- Cada modo cuenta con dos versiones (menos el proyecto), las cuales son funcionalmente iguales, la diferencia está en la telemetría que se implementa (según especificaciones del enunciado del proyecto para telemetria post-procesada con Matlab o nuestra versión para la telemetría en tiempo real con TelemetryViewer).

	- Dentro de cada carpeta encontramos subdirectorios para el correcto funcionamiento con PlatformIO. Los ficheros fuente se encuentran en:

			- /src : main.cpp (Fichero principal del código del modo concreto)

			- /lib/ModulesCarDrivers : Archivos .cpp y .hpp desarrollados de librería propia.

			* Resto de librerias contenidas en /lib son externas de terceros, necesarias para el correcto funcionamiento (timers y servos).

- Si se desea trabajar con la IDE Arduino es compatible. Debe cambiarse la extensión de main: de main.cpp a main.ino. Los códigos .cpp y .hpp de las librerias deben estar contenidas en la misma carpeta que main.ino en dicho caso.