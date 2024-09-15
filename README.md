# Tutorial de TIAGo en ROS 2 - Crea tu propia motion

## Docker
### Instalación
Antes de comenzar, necesitamos poder ejecutar aplicaciones gráficas dentro de un docker de ROS 2 Humble.

Para ello, lo primero que necesitaremos es instalar `docker`. Puedes encontrar las instrucciones de instalación [aqui](https://docs.docker.com/engine/install/ubuntu).

Una vez instalado, para poder utilizar docker sin el usuario `root`, seguiremos las siguientes [instrucciones](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

Finalmente, deberíamos ser capaces de ejecutar el siguiente comando sin errores:
```bash
docker run hello-world
```

### Ejecución de aplicaciones gráficas: pal_docker.sh

Ya con `docker` instalado, necesitaremos poder ejecutar aplicaciones gráficas dentro de él. Para conseguirlo, necesitaremos clonar el repositorio [pal_docker_utils](https://github.com/pal-robotics/pal_docker_utils), que contiene un script que permite arrancar un docker con una configuración específica. Además, permite la ********************************************************************************************************************

Para arrancar un docker de ROS 2 Humble, utilizaremos el siguiente comando:
```bash
PATH_TO_SCRIPTS_DIR/pal_docker.sh --name roscon-es-2024-tiago-workshop -it ros:humble bash
```

Y dentro del docker ejecutaremos lo siguiente:
```bash
apt-get update && apt-get install mesa-utils -y
```

Para comprobar que podemos ejecutar aplicaciones gráficas dentro del docker, ejecutamos el siguiente comando:
```bash
glxgears
```

### Ejecución de terminales
Para abrir múltiples terminales con el mismo docker, tenemos varias opciones:

#### Bash
Abrimos una nueva terminal y ejecutamos el mismo docker utilizando `bash`:
```bash
docker exec -it roscon-es-2024-tiago-workshop bash
```
De esta forma tendremos una nueva terminal dentro del mismo docker.

#### Terminator
Necesitaremos instalar `terminator` dentro del docker:
```bash
apt-get install terminator -y
```

<!-- TODO split posibilities better -->
Podemos utilizar las siguientes opciones:
1. En la terminal que tenemos abierta ejecutamos:
```bash
terminator -u
```
De este modo, esa terminal quedará bloqueada con la ejecución de este comando, pero tendremos otra terminal de la que podemos crear nuevas ejecutando `Ctrl+E` o `Ctrl+O`.

2. Abrimos una nueva terminal y ejecutamos el mismo docker directamente utilizando `terminator`:
```bash
docker exec -it roscon-es-2024-tiago-workshop terminator
```
El comportamiento será similar a la opción anterior, usaremos `Ctrl+E` o `Ctrl+O` para crear nuevas terminales.

## Workspace
Una vez que tenemos el docker configurado, iremos al directorio `/home/user/exchange`. Todo lo que creemos en ese directorio, lo tendremos localmente en el directorio `~/exchange` de nuestra máquina.
```bash
cd /home/user/exchange
```

Ahora instalaremos algunos paquetes que necesitaremos durante el proceso:
```bash
apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
```

Crearemos un workspace y clonaremos todos los repositorios necesarios para el workshop:
```bash
mkdir -p create_your_motion_ws/src
cd create_your_motion_ws
vcs import --input https://raw.githubusercontent.com/Noel215/roscon-es-2024-tiago-workshop/main/roscon-es-2024-tiago-workshop.repos src
```

Después, necesitamos instalar todas las dependencias utilizando `rosdep`:
```bash
rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Haremos source de ROS 2 y compilamos el workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Una vez compilado, haremos source del workspace, preferiblemente en una terminal diferente, [como recomienda la documentación de ROS 2](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay): 
```bash
source install/setup.bash
```

## Crea tu propia motion
### Configuración del entorno
Antes de proceder con la ejecución de aplicaciones en ROS 2, vamos a configurar el entorno. Necesitaremos ejecutar estos comandos en cada una de las terminales del docker con las que trabajemos.
```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=101
```

Además, tendremos que habilitar el multicast en la interfaz `lo`. Esto sólo necesitaremos hacerlo una vez.
```bash
ip link set lo multicast on
```

### Ejecución
Para lanzar todas las aplicaciones a la vez, ejecutaremos [el launcher de este repositorio](create_your_motion/launch/create_your_motion.launch.py).
```bash
ros2 launch create_your_motion create_your_motion.launch.py
```
