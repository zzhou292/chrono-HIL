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
  ChTCPServer server(1209, 3);

  server.Initialize();

  sleep(5);

  std::vector<float> write_data;
  write_data.push_back(1.f);
  write_data.push_back(2.f);
  write_data.push_back(3.f);

  server.Write(write_data);

  return 0;
}