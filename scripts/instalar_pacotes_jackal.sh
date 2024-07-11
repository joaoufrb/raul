echo "Lembre-se de rodar este script utilizando sudo!"
echo "Atualizando o repositório apt..."
apt-get update
echo "Instalando o pacote ROS para o Jackal..."
apt install ros-noetic-jackal-desktop 
echo "Instalando o pacote ROS para simulação do Jackal no Gazebo..."
apt install ros-noetic-jackal-gazebo 

echo "Instalando biblioteca python para detecção de April Tags pela câmera..."
pip install apriltag

# install packages for Intel RealSense D435 camera (realsense2_camera and realsense2_description).
echo "Instalando pacote de simulação da câmera Intel RealSense D435..."
apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-gazebo-plugins
