# Hera Robot

## Instalação
1. Clone este repositório.
```
cd src
git clone https://github.com/robofei-home/hera_robot
``` 
2. Instale todas as dependências.
```
cd ~/<catkin_workspace>/src/hera_robot/hera_description
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
```
3. Compile seu espaço de trabalho catkin.
```
cd  <catkin_workspace>/
catkin_make
source devel/setup.bash
```
4. Agora está pronto para usar o roslaunch para iniciar o robô simulado.
```
roslaunch hera_description visualize_model.launch
```
