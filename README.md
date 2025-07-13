Para rodar o corredor com o robô e as tags
```bash
roslaunch professor_mundo corredor.launch
```
Para rodar o nó de teste da câmera
```bash
rosrun p3dx_visao recebe_camera_teste.py
```
Para rodar o nó que controla o robô com o teclado
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=p3dx/cmd_vel 
```
