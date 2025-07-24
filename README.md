### Para rodar o corredor com o robô, as tags posicionadas e o teleop
```bash
roslaunch professor_mundo corredor.launch
```
### Para rodar o nó de teste da câmera
```bash
rosrun p3dx_visao recebe_camera_teste.py
```

### Para rodar o localizador aruco
```bash
roslaunch localizacao_aruco localizacao_aruco.launch
```
### Pode ser necessário instalar algumas dependências para rodar o pacote lidar_p3dx
```bash
rosdep install --from-paths src --ignore-src -r -y
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
cd ~/catkin_ws/src/robotica_movel_simulacao/professor_teste/lidar_p3dx/maps
rosrun map_server map_saver -f corredor_map_obstaculo
```

### Para usar o AMCL para estimar a pose do robô no mapa utilizando o LiDAR
1. Rodar o corredor.launch
2. Rodar o localization.launch
```bash
roslaunch lidar_p3dx localization.launch
```

