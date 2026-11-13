sudo ifconfig can0 down
sudo ip link set can0 up type can onbitrate 1000000 berr-reporting on
sudo ip link set can0 up
