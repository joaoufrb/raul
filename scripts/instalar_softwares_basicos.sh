echo "Lembre-se de rodar este script utilizando sudo!"
echo "Atualizando o repositório apt..."
apt-get update
echo "Instalando curl..."
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "Atualizando o repositório apt..."
apt update
echo "Instalando ROS completo..."
apt install ros-noetic-desktop-full
echo "Instalando pacote para auxiliar visualização de dados do ROS..."
apt install ros-noetic-rqt

echo "Instalando gerenciador de pacotes do python..."
apt install pip
echo "Instalando editor de texto GVIM..."
apt install vim-gtk3
echo "Instalando reprodutor de vídeo VLC..."
apt install vlc
echo "Instalando xterm (para rodar múltiplos terminais a partir de um script..."
apt install xterm
echo "Instalando git..."
apt install git
