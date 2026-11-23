#!/bin/bash
set -e

IMAGE_NAME="stochastic-maze-autopilot:noetic"
CONTAINER_NAME="maze_autopilot_sim"

echo "1. Permitiendo acceso gráfico X11..."
xhost +local:docker > /dev/null 2>&1 || true

# Construir la imagen solo si no existe en la PC del usuario
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
  echo "2. Construyendo entorno Docker con ROS Noetic y TurtleBot3 (solo la primera vez)..."
  docker build -t $IMAGE_NAME .
fi

# Eliminar contenedor previo si quedó detenido
docker rm -f $CONTAINER_NAME > /dev/null 2>&1 || true

echo "3. Iniciando simulación autónoma en RViz..."
docker run -it --rm --name $CONTAINER_NAME \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/workspace:rw" \
  $IMAGE_NAME \
  bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch /workspace/simulation/simulation.launch"
