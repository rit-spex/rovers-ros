To use autoBoot we will need to move the .service file. Before doing this ensure that all of the path are correct.
First move the .service file to /etc/systemd/system directory
sudo cp autoBoot.service /etc/systemd/system/autoBoot.service

Next call start and enable to have it work on startuo
sudo systemctl start autoBoot.service
sudo systemctl enable autoBoot.service

#to check if the service is working
#sudo systemctl daemon-reload
#systemctl status autoBoot.service
