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
// This is a stream-based input driver interface based on boost UDP networking
// =============================================================================

#ifndef CH_BOOST_DATASTREAMER_H
#define CH_BOOST_DATASTREAMER_H

#include <string>

#include "../../ChApiHil.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

namespace chrono {
namespace hil {

// Driver for the leader vehicle, it adjusts its target speed according to a
// piecewise sinusoidal function In the buffer-areas between pieces it keeps the
// target speed specified in target_speed
class CH_HIL_API ChBoostOutStreamer {
public:
  /// Construct an interactive driver.
  ChBoostOutStreamer(std::string end_ip_addr, int port);

  ~ChBoostOutStreamer() { m_socket->close(); };

  void AddData(float data_in);

  void Synchronize();

private:
  std::shared_ptr<boost::asio::io_service> m_io_service;
  std::shared_ptr<boost::asio::ip::udp::socket> m_socket;
  std::shared_ptr<boost::asio::ip::udp::endpoint> m_remote_endpoint;
  std::vector<float> m_stream_data;
  std::string m_end_ip_addr;
  int m_port;
};

} // namespace hil
} // namespace chrono
#endif