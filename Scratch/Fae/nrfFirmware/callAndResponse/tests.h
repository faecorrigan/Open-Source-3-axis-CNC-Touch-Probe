

#define TESTSPAIRING set_radio_mode(SEND); \
  build_packet(0x03, 0xea, 0x0b); \
  send_packet(packet); \
  set_radio_mode(RECIEVE); \
  if (receive_test() == 2){ \
    delay(1500); \
    set_radio_mode(SEND); \
    build_packet(0x06, 0xea, 0x0b); \
    send_packet(packet);\
  }\
  delay(100);