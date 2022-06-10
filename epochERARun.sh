
cd src;
python read_bag_1.py ../data/2020-09-10-14-43-09.bag &> read_bag_1.out &
python read_bag_2.py ../data/2020-09-10-14-43-09.bag &> read_bag_2.out &
./wifi_comm_1.sh &> wifi_comm_1.out &
./wifi_comm_2.sh &> wifi_comm_2.out &
./carla_recvr_1.sh &
./carla_recvr_2.sh &
cd ..;
