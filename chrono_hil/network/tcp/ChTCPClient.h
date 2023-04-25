// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
// This is a stream-based input driver interface based on boost TCP networking
// =============================================================================

#ifndef CH_TCP_CLIENT_H
#define CH_TCP_CLIENT_H

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
class CH_HIL_API ChTCPClient {
public:
  ChTCPClient(std::string ip_addr, int port_in, int data_len); // create

  ~ChTCPClient();

  void Initialize(); // create socket and send signal to acceptor

  int Write(std::vector<float> write_data);

  int Read();

  std::vector<float> GetRecvData() { return m_recv_stream_data; }

private:
  std::shared_ptr<boost::asio::io_service> m_io_service;
  std::shared_ptr<boost::asio::ip::tcp::endpoint> m_tcpendpt;
  std::shared_ptr<boost::asio::ip::tcp::socket> m_socket;
  std::vector<float> m_recv_stream_data;
  int m_port; // fixed port connection
  int m_len;  // fixed receive data length
  std::string m_addr;
};

} // namespace hil
} // namespace chrono
#endif