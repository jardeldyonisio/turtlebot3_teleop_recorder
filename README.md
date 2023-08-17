# turtlebot3_teleop_recorder

Esse pacote é utilizado para gravar a trajetória realizada pela turtlebot3 e mostrar no Rviz. Em primeiro momento está funcional para ROS versão Noetic, confira abaixo como vocẽ pode testar o pacote.

## Dependencias

- Ubuntu;
- [ROS Noetic;](http://wiki.ros.org/noetic/Installation/Ubuntu)

Clone na pasta src que está no seu catkin_ws
```
git clone https://github.com/jardeldyonisio/turtlebot3_teleop_recorder.git
```

## Preparando o pacote

Compile o package no seu catkin_ws
```
catkin_make
```

## Passos para testar

Siga as instruções para gravar e carregar o path realizado pelo turtlebot3.

### Executando o turtlebot3

Rode o launch do turtlebot3 no mapa world:
```
roslaunch turtlebot3_teleop_recorder world_navigation.launch
```
Agora rode o launch responsável por mover o turtlebot3 utilizando o teclado:
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Dessa forma, você vai poder controlar o robô pelo mapa.

### Executando os scripts

Para começar a gravar a trajetória feita pelo robô e execute o comando:
```
rosrun turtlebot3_teleop_recorder recorder_points.py
```
Quando você não desejar mais gravar o path, você pode encerrar com as teclas CTRL + C

Para mostrar no Rviz o path executado pelo turtlebot3, você pode rodar o seguinte comando na pasta scripts:

```
rosrun turtlebot3_teleop_recorder points_visualizer.py
```

No Rviz, você deve ir em 'add' e selecionar o marker para aparecer a trajetória gravada.

[Confira aqui o vídeo de funcionamento do pacote](https://www.youtube.com/watch?v=pz3rTLs5pZE)

<p align="center"> 
  <i>If you liked this repository, please don't forget to starred it!</i>
  <img src="https://img.shields.io/github/stars/jardeldyonisio/turtlebot3_teleop_recorder?style=social"/>
</p>
