mkdir -p ~/bagfiles
cd ~/bagfiles
sudo tcpdump -i ens33  port 3002 -w capture.pcap


