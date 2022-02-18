[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870051&assignment_repo_type=AssignmentRepo)
# fsm_bump_go
## (CC-BY-NC-SA) Estefanía Ochoa Plaza, Elisa Jazmín Matos Juarez, José Miguel Jiménez García y Adrián Cobo Merino.

El objetivo de este esta práctica es tener la primera toma de contacto con los lasers y bumpers del kobuki.

Para realizar los ejecicios hemos programado una maquina de estados que va a avanzar, retroceder y girar segun las condiciones que se requiera en cada programa.

**Antes de continuar:**

Es importante saber que el laser tiene un rango de 360º y  por ello vamos a filtrar las medidas para quedarnos con las que nos interesan.
Hemos creado una clase Base de la que va a herededar sus métodos y atributos la clase Bumpandgo_Advanced_Laser y Bumpandgo_Advanced.
Lo único que cambia en el constructor es el topic al que se subscriben.

Hemos encontrado alguna dificultad a la hora de establecer los limites entre izquierda, derecha y delante.
Para solucionarlo hemos decidido dividir el espacio en tres regiones como se puede observar en la imagen.

También hemos tenido un pequeño problema con el láser, ya que detectaba objetos detrás y se iba hacia atrás constantemente.
Para solventar el problema en vez de recorrer el array de posiones entero hemos recorrido un cuarto del array en un for y el lado contrario con otro for
para quedarnos con las posiciones que nos interesan.


Dejamos aquí unos videos del funcionamiento de cada uno de los programas:

-basic_bumpandgo, pulsa [aqui](https://drive.google.com/file/d/1GnxpmbcihN2C3uI69yehvSQSALo0_TUO/view?usp=sharing). 
-advanced_bumpandgo, pulsa [aqui](https://drive.google.com/file/d/1Ezp-X9_mWszJ-PYYbqp3Tguup_katKey/view?usp=sharing). 
-advanced_laser, pulsa [aqui](https://drive.google.com/file/d/1THWqhBqXEmvrdnSB0w2TBFtgH_1oeeKy/view?usp=sharing). 

Para cualquier duda escribidnos por git.
