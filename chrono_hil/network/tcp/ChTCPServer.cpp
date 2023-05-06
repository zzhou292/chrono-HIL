// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
// This is a stream-based input driver interface based on boost TCP networking
// =============================================================================

#include "ChTCPServer.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace chrono {
namespace hil {

ChTCPServer::ChTCPServer(int port_in, int data_len) {
  m_port = port_in;
  m_len = data_len;
  m_recv_stream_data.clear();
}

ChTCPServer::~ChTCPServer() {}

void ChTCPServer::Initialize() {
  m_io_service = std::make_shared<boost::asio::io_service>();
  m_tcpendpt = std::make_shared<boost::asio::ip::tcp::endpoint>(
      boost::asio::ip::tcp::v4(), m_port);
  m_acceptor = std::make_shared<boost::asio::ip::tcp::acceptor>(*m_io_service,
                                                                *m_tcpendpt);

  m_socket = std::make_shared<boost::asio::ip::tcp ::socket>(*m_io_service);
  m_acceptor->accept(*m_socket);
}

int ChTCPServer::Write(std::vector<float> write_data) {
  boost::asio::write(*m_socket,
                     boost::asio::buffer(write_data.data(),
                                         sizeof(float) * write_data.size()));
  return 0;
}

int ChTCPServer::Read() {

  float tcp_float_arr[m_len];

  // m_recv_stream_data.clear();
  boost::asio::read(*m_socket,
                    boost::asio::buffer(&tcp_float_arr, m_len * sizeof(float)));
  m_recv_stream_data.clear();
  for (int i = 0; i < m_len; i++) {
    m_recv_stream_data.push_back(tcp_float_arr[i]);
  }

  return 0;
};

} // namespace hil
} // namespace chrono