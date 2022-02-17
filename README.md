[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870051&assignment_repo_type=AssignmentRepo)
# fsm_bump_go
## (CC-BY-NC-SA) Estefanía Ochoa Plaza, Elisa-Jazmín Matos Juarez, Jose Miguel y Adrián Cobo Merino.

El objetivo de este esta práctica es tener la primera toma de contacto con los lasers y bumpers del kobuki.

Para realizar los ejecicios hemos programado una maquina de estados que va a avanzar, retroceder y girar segun las condiciones que se requiera en cada programa.

**Antes de continuar:**

Es importante saber que el laser es 360º y que por ende vamos a tener que filtrar las medidas que realmente nos interesen. Ademas vamos a heredar la maquina de estados de la clase Bumpandgo_Advanced para el Bumpandgo_advancedLaser ya que lo unico que vamos  a necesitar cambiar realmente es el topic al que vamos a estar subscritos(bumpero o laser) y por ende las condiciones que van a cambiar cada estado de la maquina.

Importante cambiar los links.
Para ver el video 1 sobre el sensor de presencia, pulsa [aqui](https://drive.google.com/file/d/1eGHyPk4TzpwS6tXNx0YoAtnOmYzHVtVp/view?usp=sharing).
Para ver el video 2 sobre el sensor de presencia, pulsa [aqui](https://drive.google.com/file/d/1b3hyRx6AsqsTvuRgkNGxKWlDZyfwUoql/view?usp=sharing).
Para ver el video 3 sobre el sensor de presencia, pulsa [aqui](https://drive.google.com/file/d/1b3hyRx6AsqsTvuRgkNGxKWlDZyfwUoql/view?usp=sharing).

Para cualquier duda escribidnos por git.
