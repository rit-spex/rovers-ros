sudo ifconfig can0 down
sudo ip link set can0 up type can berr-reporting onbitrate 1000000
sudo ip link set can0 up
