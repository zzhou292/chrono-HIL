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
#include "chrono/core/ChVector.h"

using boost::asio::ip::address;
using boost::asio::ip::udp;

namespace chrono {
namespace hil {

<<<<<<< HEAD

struct ChronoVehicleInfo
{
    int vehicle_id;
    long long time_stamp;
    double position[3];
    double orientation[3];
    double steering_angle;
    double wheel_rotations[4];
=======
struct ChronoVehicleInfo {
  long long vehicle_id;
  long long time_stamp;
  long long position[3];
  long long orientation[3];
  long long steering_angle;
  long long wheel_rotations[4];
>>>>>>> main
};

// Driver for the leader vehicle, it adjusts its target speed according to a
// piecewise sinusoidal function In the buffer-areas between pieces it keeps
// the target speed specified in target_speed
class CH_HIL_API ChBoostOutStreamer {
public:
  /// Construct an interactive driver.
  ChBoostOutStreamer(std::string end_ip_addr, int port);

  ~ChBoostOutStreamer() { m_socket->close(); };

  void AddData(float data_in);

<<<<<<< HEAD
  void AddVector(ChVector<float> data_in);

  void AddLongLongData(long long data_in);

  void AddLongLongVector(ChVector<long long> data_in);

  void AddChronoVehicleInfo(ChronoVehicleInfo data_in);
=======
  void AddVehicleStruct(ChronoVehicleInfo info);
>>>>>>> main

  void Synchronize();

private:
  std::shared_ptr<boost::asio::io_service> m_io_service;
  std::shared_ptr<boost::asio::ip::udp::socket> m_socket;
  std::shared_ptr<boost::asio::ip::udp::endpoint> m_remote_endpoint;
  std::vector<float> m_stream_data;
<<<<<<< HEAD
  std::vector<long long> m_long_long_stream_data;
  std::vector<ChronoVehicleInfo> m_vehicle_data;
=======
  std::vector<ChronoVehicleInfo> m_stream_vehicle_data;
>>>>>>> main
  std::string m_end_ip_addr;
  int m_port;
};

} // namespace hil
} // namespace chrono
#endif