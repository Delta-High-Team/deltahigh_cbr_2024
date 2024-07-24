# Raspberry Pi Camera - Node publisher

# Install v4l2
sudo apt install libraspberrypi-bin v4l-utils ros-rumble-v4l2-camera ros-rumble-image-transport-plugins

# Verifique se tem permissao de acesso para cameras
groups

# Deve aparecer "video" (ex: deltahigh adm tty dialout cdrom floppy sudo audio dip video plugdev netdev lxd)
# Se nao
sudo usermod -aG video $ubuntu

# Verifique se a camera foi detectada (deve retornar: supported=1 detected=1)
vcgencmd get_camera

# Verifique se o V4L2 consegue acessar a camera
v4l2-ctl --list-devices

# Deve retornar algo do tipo:
# mmal service 16.1 (platform:bcm2835-v4l2-0):
#         /dev/video0

# Para iniciar o node (deve estar na pasta do camera.launch.py)
ros2 launch camera.launch.py