# turtlebot3_teleop_recorder

Esse pacote é utilizado para gravar a trajetória realizada pela turtlebot3 e mostrar no Rviz. Em primeiro momento está funcional para ROS versão Noetic, confira abaixo como vocẽ pode testar o pacote.

## Dependencias

- Ubuntu;
- [ROS Noetic;](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Pacotes do Turtlebot3.](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

Clone na pasta src que está no seu catkin_ws
```
git clone https://github.com/jardeldyonisio/turtlebot3_teleop_recorder.git
```

## Preparando o pacote

Acesse a pasta
```
cd ~/catkin_ws/src
```

Torne executável os scripts:
```
sudo chmod +x recorder.py
sudo chmod +x trajectory_visualizer.py
```
## Passos para testar

Siga as instruções para gravar e carregar o path realizado pelo turtlebot3.

### Executando o turtlebot3

Carregue o robô burguer:
```
export TURTLEBOT3_MODEL=burger
```

Rode o launch do turtlebot3 no mapa world:
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Agora rode o launch responsável por mover o turtlebot3 utilizando o teclado:
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Executando os scripts

Para começar a gravar a trajetória feita pelo robô, acesse a pasta script e execute o comando:
```
./recorder.py
```
Quando você não desejar mais gravar o path, você pode encerrar com as teclas CTRL + C

Para mostrar no Rviz o path executado pelo turtlebot3, você pode rodar o seguinte comando na pasta scripts:

```
./trajectory_visualizer.py
```

[Confira aqui o vídeo de funcionamento do pacote](https://youtu.be/eSORJADcSzg)

<p align="center"> 
  <i>If you liked this repository, please don't forget to starred it!</i>
  <img src="https://img.shields.io/github/stars/jardeldyonisio/turtlebot3_teleop_recorder?style=social"/>
</p>