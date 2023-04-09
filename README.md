# Workspace for EL5206 Course - Robotics Component

FCFM - Universidad de Chile - Depto. Ingeniería eléctrica

Autores: Felipe Vergara, Ignacio Dassori, Eduardo Jorquera
 
Este es el workspace de ROS que se utilizará para el semestre Primavera 2023 de EL5206. 

### Requisitos:
Sistema operativo Ubuntu 20.04

# Instrucciones de instalación y uso 

Para la instalación, se debe clonar este repositorio en su directorio HOME:

```sh
    cd ~
    git clone https://github.com/Uchile-Lab-Vision-Computacional/delphi_radars.git
```

Luego, se entra a la carpeta `el5206_ws` y se debe hacer `source` al archivo `install.bash` que correrá todos los comandos necesarios para la instalación y setup del workspace del curso.

```sh
    cd el5206_ws
    source install.bash
```

Luego de esto, se deberían contar con los programas y paquetes necesarios para realizar las actividades del curso.

## Uso de launch files

En el directorio `/el5206_ws/src/el5206_gazebo/launch`, se encuentran los archivos `.launch`, los cuales lanzan las simulaciones en el ambiente del programa Gazebo. En particular, los que se utilizarán son:

* `el5206_empty_world.launch` que lanzará el robot en un mundo vacío.
* `el5206_house.launch` que lanzará el robot en el mapa `turtlebot3_house`.

Para correr los archivos launch, se hace como sigue:

```sh
    roslaunch el5206_gazebo el5206_house.launch 
```
Donde el primer término es el comando para correr los archivos launch, el segundo es el nombre del paquete de ros donde está el archivo y el tercero es el nombre de este.

## Correr archivos de python o ejecutables

### Ejemplo Teleoperación manual del robot

Para correr scripts/nodos durante la ejecución de la simulación se debe utilzar el comando `rosrun`. Similar a la lógica de los archivos launch, se debe correr el comando, seguido del nombre del paquete de ROS donde se ubica lo que se quiere correr y luego el nombre del archivo mismo.

```sh
rosrun turtlebot3_teleop turtlebot3_teleop_key 
```

> __Note__: El archivo debe ser del tipo ejecutable para poder ser utilizado por `rosrun`. Si no lo es, se debe ir al directorio donde se encuentra desde la terminal y correr:
> `chmod +x <NOMBRE_DEL_ARCHIVO>`

### Script para laboratorios del curso 
El principal script donde se harán los laboratorios del curso es un archivo de python, que se encuentra en el paquete `el5206_example` y se llama `el5206_main.py`. 

Se encuentra en la carpeta `/el5206_ws/src/el5206_example/scripts`.






