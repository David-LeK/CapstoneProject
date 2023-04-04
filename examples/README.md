```
sudo apt install ros-noetic-geographic-*
sudo apt install geographiclib-*
sudo apt install libgeographiclib-*
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-*/Modules/
```

```
g++ -o convert convert.cpp -lGeographic
```
