// #include "../src/treasure_map/include/TreasureMap.h"
#include "../src/uart/include/uart.h"

int main() {
  uart::SerialPort serial_ = uart::SerialPort(
      "/home/ling/Photoelectric_Competition/config/uart.xml");
  int i = 0;

  std::vector<std::pair<uint8_t, uint8_t>> keypoint;
  keypoint.push_back(std::pair<uint8_t, uint8_t>(12, 32));
  keypoint.push_back(std::pair<uint8_t, uint8_t>(15, 42));
  keypoint.push_back(std::pair<uint8_t, uint8_t>(16, 62));
  keypoint.push_back(std::pair<uint8_t, uint8_t>(14, 25));
  while (1) {
    // serial_.writeData(int(100 + i), int(200 + i));
    serial_.writeData(keypoint);

    usleep(1000000);

    // serial_.receiveData2();
    // usleep(1000000);
    i++;
  }

  return 0;
}