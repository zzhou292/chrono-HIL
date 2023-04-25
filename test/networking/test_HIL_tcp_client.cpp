#include <chrono>
#include <iostream>
#include <stdint.h>
#include <thread>

#include "chrono_hil/network/tcp/ChTCPClient.h"
#include "chrono_hil/network/tcp/ChTCPServer.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::hil;

int main(int argc, char *argv[]) {
  ChTCPClient client("127.0.0.1", 1209, 3);
  client.Initialize();

  client.Read();

  std::vector<float> temp = client.GetRecvData();
  for (int i = 0; i < temp.size(); i++)
    std::cout << temp[i] << std::endl;
  return 0;
}