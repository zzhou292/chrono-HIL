// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// Jason Zhou
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// This is a stream-based input driver interface based on boost UDP networking
// =============================================================================

#include "ChBoostInStreamer.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace chrono {
namespace hil {

ChBoostInStreamer::ChBoostInStreamer(int port_in, int data_len) {
  m_port = port_in;
  m_len = data_len;
  m_recv_stream_data.clear();

  m_io_context = std::make_shared<boost::asio::io_context>();
  m_udpendpt =
      std::make_shared<boost::asio::ip::udp::endpoint>(udp::v4(), m_port);
  m_socket = std::make_shared<boost::asio::ip::udp::socket>(*m_io_context,
                                                            *m_udpendpt);
}

ChBoostInStreamer::~ChBoostInStreamer() {}

int ChBoostInStreamer::Synchronize() {
  float udp_float_arr[m_len];

  m_recv_stream_data.clear();

  if (m_socket->receive_from(
          boost::asio::buffer(&udp_float_arr, sizeof(float) * m_len),
          *m_udpendpt)) {
    for (int i = 0; i < m_len; i++) {
      m_recv_stream_data.push_back(udp_float_arr[i]);
    }
  }

  return 0;
}

} // namespace hil
} // namespace chrono