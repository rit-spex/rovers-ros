This will show you how to set up the start up software.

To use auto_boot we will need to move the .service file. Before doing this ensure that all of the path are correct.
First move the .service file to /etc/systemd/system directory
sudo cp auto_boot.service /etc/systemd/system/auto_boot.service

Next call start and enable to have it work on startup
sudo systemctl start auto_boot.service
sudo systemctl enable auto_boot.service

#to check if the service is working
#sudo systemctl daemon-reload
#systemctl status auto_boot.service
