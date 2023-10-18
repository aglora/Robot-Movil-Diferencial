# Robot-Movil-Diferencial
Proyecto de construcción hardware y software de un robot móvil con ruedas en configuración diferencial y el control realizado del mismo para conseguir distintos modos de funcionamiento mediante realimentación sensorial.

<div style="display: flex; flex-direction: row;">
  <img src="https://github.com/aglora/Robot-Movil-Diferencial/blob/main/FotoRobotMovil.jpg" width="500" />
  <img src="https://github.com/aglora/Robot-Movil-Diferencial/blob/main/Vista-aerea-robot.jpg" width="500" />
</div>

Toda la documentación está detallada en la memoria de trabajo, en la cual se explican las técnicas de control implementadas, esquemas usados, montaje físico y explicaciones principales de los códigos usados.

RESUMEN:

MODO 1:
El objetivo será conseguir que el vehículo se pare frente a una pared a una
cierta referencia de distancia fijada. Para ello únicamente contaremos con un
sensor de ultrasonido con el que realizaremos las medidas de distancia con las
que cerrar el bucle de control.

MODO 2:
Se pretende mejorar el modo anterior incluyendo un segundo sensor de
ultrasonido, tomándose una segunda medida de distancia con la que poder
conseguir una orientación lo más perpendicular posible a la pared frente a la
cual se mantiene la distancia fijada en la referencia que se le proporcione.

MODO 3:
El objetivo de este modo será lograr un desplazamiento paralelo a la pared
manteniendo la distancia inicial que tenga a la misma en todo momento.

MODO 4:
En esta ocasión la referencia de distancia será variable y el objetivo será que
el robot se posicione a la distancia indicada de la pared a la vez que se desplaza
paralelamente a la misma al igual que hacía en el modo anterior. En los
permanentes, por tanto, el comportamiento que se pretende obtener será
idéntico al modo 3, la diferencia radica en la habilidad para realizar las
transiciones de acercamiento y alejamiento respecto al plano de la pared.

MODO 5:
El fin de este modo será el de controlar la velocidad angular de cada rueda.
Lo haremos en revoluciones por minuto. Para ello se instalarán sensores que
permitan medir el giro de las ruedas y a partir de esto hacer una estimación
de la velocidad.

MODO 6:
Partiendo del modo anterior, se desea hacer que el robot se mueva en línea
recta a partir del control en velocidad de cada rueda.

MODO 7:
Introducimos el modelo cinemático conocido de la configuración diferencial,
que nos permitirá la obtención de una estimación de la posición basada en
odometría.
El objetivo será el movimiento del robot a posiciones deseadas respecto a un
sistema de referencia global el cual tomaremos como el punto inicial donde
posicionamos el robot al ejecutar el programa. El sistema de ejes locales estará
posicionado en el punto central sitiado entre ambas ruedas. Al hablar de la
pose del robot nos estaremos refiriendo a dicho punto.

PROYECTO FINAL:
Se pretende desarrollar un nuevo modo de alcance de puntos objetivos dados
con detección y evitación de obstáculos. También se propone una aplicación
alternativa de seguimiento de carriles con paredes.

<div style="display: flex; flex-direction: row;">
  <img src="https://github.com/aglora/Robot-Movil-Diferencial/blob/main/Conexionado-Proyecto.png" width="400" />
  <img src="https://github.com/aglora/Robot-Movil-Diferencial/blob/main/TelemetryViewer.jpg" width="400" />
</div>
