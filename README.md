[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870051&assignment_repo_type=AssignmentRepo)
# fsm_bump_go
# P9-Fuerza-Presencia

## (CC-BY-NC-SA) Adrián Cobo Merino

El objetivo de este esta práctica es tener la primera toma de contacto con los sensores de presión y de presencia.

Para realizar los ejecicios de presión.py y presencia.py hemos modificado un poco el programa de la práctica anterior pues el comportamiento 
era bastante similar. Aun así nos ha servido para introducirnos estos dos sensores y poder hacer el ejercicio del jardín inteligente.

**Antes de continuar:**

Es importante saber que el sensor de proxidad tiene 2 tornillos con los cuales puede ser regulado. Si ponemos el sensor con la esfera mirando
hacia abajo, el tornillo de la izquierda nos sirve para regular la sensibilidad del sensor, y el de la derecha para regular un temporizador 
para enviar 1 o más señales de salida(según se ajuste el jumper del sensor explido posteriormente) en el tiempo seleccionado. 

El sensor cuenta con un jumper.En la posición más esquinada el sensor solo enviara un pulso de salida pese a detectar mas presencias
durante el bouncetime ajustado en el tornillo. Cambiandolo de posición,sí que se generaría un nuevo evento por cada detección
de movimiento.

**Se recomienda al usuario que vaya ajustando estos 3 elementos del sersor de proximidad para poder cumplir con el requerimiento deseado.** 

Además, he subido dos videos de ejemplo sobre el sensor de presencia. En el primero vemos como el sensor no detecta la presencia
correctamente pues pusimos que tubiese la menos sensibilidad posible y que el cronómetro del sensor tubiese un valor máximo. Además el 
jumper estaba colocado para que solo enviase un pulso durante el tiempo indicado por el tornillo de la derecha si detectaba presencias.
En el segundo,vemos como el sensor ya funciona correctamente al ajustar los tornillos.

Para ver el video 1 sobre el sensor de presencia, pulsa [aqui](https://drive.google.com/file/d/1eGHyPk4TzpwS6tXNx0YoAtnOmYzHVtVp/view?usp=sharing).
Para ver el video 2 sobre el sensor de presencia, pulsa [aqui](https://drive.google.com/file/d/1b3hyRx6AsqsTvuRgkNGxKWlDZyfwUoql/view?usp=sharing).

**El esquema de conexión es igual que el facilitado en el enunciado a diferencia de lo que se puede intuir de**

```python
   
   sensorPresencia = 16
   
   #GPIO Mode (BOARD / BCM)
   GPIO.setmode (GPIO.BCM)
```
**Para el sensor de presión tambien sigue el mismo esquema**

Si quieres ver un video de demostración del programa "presión.py", pulsa [aqui](https://drive.google.com/file/d/1o0RCPiRUgLOWhlj3wLzQ05gJH_ioiWLC/view?usp=sharing).

Para realizar el ejercicio de jardin_inteligente.py hemos combinado los 2 programas anteriores y además hemos incluido un cronómetro que 
modificará el valor de un flag. 

Al presionarse el sensor de presión, el flag se activará y se arrancará el cronometro haciendo que si el sensor de presencia detecta algo, 
solo se encienda un led si el valor del cronómetro es menor que 30 segundos. Finalmente cuando el cronometro llegue a 30 segundos, 
el flag se desactivará y aunque detecte presencia el sensor, no se iluminará el led hasta que el sensor de presión vuelva a ser presionado.

Además añadir que hemos utilizado varios hilos para poder realizar el programa.

**El esquema de coneión es igual que el facilitado en el enunciado a diferencia de lo que se puede intuir de**

```python
   
	sensorPresencia = 16
	sensorPresion = 20
	ledRojo=2
	
	#GPIO Mode (BOARD / BCM)
    GPIO.setmode (GPIO.BCM)
	
```

Si quieres ver un video de demostracion del programa, pulsa [aqui](https://drive.google.com/file/d/1w_W2qdlp-PyUKYmtU29bG8oqfgi59ctj/view?usp=sharing).

Para cualquier duda: <a.cobo.2020@alumos.urjc.es>
