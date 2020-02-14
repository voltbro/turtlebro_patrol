# Turtlebro_patrol пакет робота патрульного

### Установка пакета
На RaspberryPi установить пакет "стандартным" способом

```
cd ros_catkin_ws/src
git clone https://github.com/voltbro/turtlebro_patrol
cd ..
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=turtlebro_patrol
```

### Зависимости
Для выполнения задач по патрулированию, необходмо чтобы на роботе были установленны пакеты навигации:

* turtlebro_navigation
* amcl
* dwa_local_planner
* global_planner
* gmapping
* map_server
* move_base
* move_base_msgs
