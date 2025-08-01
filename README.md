### Para rodar o corredor com o robô, as tags posicionadas e o teleop
```bash
roslaunch professor_mundo corredor.launch
```

### Para gerar um novo mapa:
1. Rodar o corredor.launch
2. Rodar o slam.launch
```bash
roslaunch lidar_p3dx slam.launch
```
3. Andar com o robô ao longo do corredor
4. Salvar o mapa com o comando:
```bash
rosrun map_server map_saver -f ~/catkin_ws/src/robotica_movel/professor_teste/lidar_p3dx/maps/corredor_map
```

### Para rodar o corredor com o robô, as tags posicionadas e os pacotes amcl, aruco e ekf juntos:
```bash
roslaunch professor_mundo corredor_pacotes.launch
```


