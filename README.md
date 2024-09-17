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
Ya con `docker` instalado, necesitaremos poder ejecutar aplicaciones gráficas dentro de él. Para conseguirlo, necesitaremos clonar el repositorio [pal_docker_utils](https://github.com/pal-robotics/pal_docker_utils), que contiene un script que permite arrancar un docker con una configuración específica. Además, permite el uso de GPU dentro del docker.

Para arrancar un docker de ROS 2 Humble, utilizaremos el siguiente comando:
```bash
PATH_TO_SCRIPTS_DIR/pal_docker.sh --name roscon-es-2024-tiago-workshop -it noeljimenez/roscon-es-2024:tiago-workshop bash
```

Para comprobar que podemos ejecutar aplicaciones gráficas dentro del docker, ejecutamos lo siguiente:
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
Podemos utilizar las siguientes opciones:
##### Opción 1
 En la terminal que tenemos abierta ejecutamos:
```bash
terminator -u
```
De este modo, esa terminal quedará bloqueada con la ejecución de este comando, pero tendremos otra terminal de la que podemos crear nuevas ejecutando `Ctrl+E` o `Ctrl+O`.

##### Opción 2
Abrimos una nueva terminal y ejecutamos el mismo docker directamente utilizando `terminator`:
```bash
docker exec -it roscon-es-2024-tiago-workshop terminator
```
El comportamiento será similar a la opción anterior, usaremos `Ctrl+E` o `Ctrl+O` para crear nuevas terminales.

## Workspace
Una vez que tenemos el docker configurado, iremos al directorio `/home/user/exchange`. Todo lo que creemos en ese directorio, lo tendremos localmente en el directorio `~/exchange` de nuestra máquina.
```bash
cd /home/user/exchange
```

Crearemos un workspace y clonaremos todos los repositorios necesarios para el workshop:
```bash
mkdir -p create_your_motion_ws/src
cd create_your_motion_ws
vcs import --input https://raw.githubusercontent.com/Noel215/roscon-es-2024-tiago-workshop/main/roscon-es-2024-tiago-workshop.repos src
```

Después, necesitamos instalar todas las dependencias utilizando `rosdep`:
```bash
apt-get update
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Haremos source de ROS 2 y compilamos el workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Una vez compilado, idealmente en una terminal diferente [como recomienda la documentación de ROS 2](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay), haremos source del workspace: 
```bash
source install/setup.bash
```

## Crea tu propia motion
### Configuración del entorno
En el docker con el que estamos trabajando, ya tenemos las siguientes variables configuradas:
```bash
ROS_LOCALHOST_ONLY=1
ROS_DOMAIN_ID=101
```

Habilitamos el multicast en la interfaz `lo`:
```bash
ip link set lo multicast on
```

### Ejecución
Para lanzar todas las aplicaciones a la vez, ejecutaremos [el launcher de este repositorio](create_your_motion/launch/create_your_motion.launch.py).
```bash
ros2 launch create_your_motion create_your_motion.launch.py
```
