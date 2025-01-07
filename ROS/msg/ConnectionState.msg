# module is currently connected (set to true when packet is sent and received. set to false when `parameter:loss_to_disconnect` packets are lost. default 1 lost packet)
bool module_connected
# number of packets dropped from send queue due to no ack received (last resort)
uint32 lost_packets_since_connect
# number of packets dropped from send queue due ever on an octet
uint32 lost_packets_accumulated