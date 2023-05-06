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
#ifndef CH_TCP_SERVER_H
#define CH_TCP_SERVER_H

#include <string>

#include "../../ChApiHil.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>

using boost::asio::ip::address;
using boost::asio::ip::udp;

namespace chrono {
namespace hil {

// Driver for the leader vehicle, it adjusts its target speed according to a
// piecewise sinusoidal function In the buffer-areas between pieces it keeps the
// target speed specified in target_speed
class CH_HIL_API ChTCPServer {
public:
  ChTCPServer(int port_in, int data_len); // create

  ~ChTCPServer();

  void Initialize(); // create acceptor and wait for connection

  int Write(std::vector<float> write_data);

  int Read();

  std::vector<float> GetRecvData() { return m_recv_stream_data; }

private:
  std::shared_ptr<boost::asio::io_service> m_io_service;
  std::shared_ptr<boost::asio::ip::tcp::acceptor> m_acceptor;
  std::shared_ptr<boost::asio::ip::tcp::endpoint> m_tcpendpt;
  std::shared_ptr<boost::asio::ip::tcp::socket> m_socket;
  std::vector<float> m_recv_stream_data;
  int m_port; // fixed port connection
  int m_len;  // fixed receive data length
};

} // namespace hil
} // namespace chrono
#endif