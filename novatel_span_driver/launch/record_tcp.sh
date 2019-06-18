mkdir -p ~/bagfiles
cd ~/bagfiles
sudo tcpdump -i eno1 port 3001 -w capture.pcap


