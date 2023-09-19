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
// This is a stream-based input driver interface based on boost UDP networking
// =============================================================================

#include "ChBoostOutStreamer.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace chrono {
namespace hil {

ChBoostOutStreamer::ChBoostOutStreamer(std::string end_ip_addr, int port) {
  m_end_ip_addr = end_ip_addr;
  m_port = port;
  m_io_service = std::make_shared<boost::asio::io_service>();
  m_socket = std::make_shared<boost::asio::ip::udp::socket>(*m_io_service);
  m_remote_endpoint = std::make_shared<boost::asio::ip::udp::endpoint>(
      address::from_string(m_end_ip_addr), m_port);
  m_socket->open(udp::v4());
}

void ChBoostOutStreamer::AddData(float data_in) {
  m_stream_data.push_back(data_in);
}

void ChBoostOutStreamer::AddVector(ChVector<float> data_in){
  m_stream_data.push_back(data_in.x());
  m_stream_data.push_back(data_in.y());
  m_stream_data.push_back(data_in.z());
}

void ChBoostOutStreamer::AddLongLongData(long long data_in){
  m_long_long_stream_data.push_back(data_in);
}

void ChBoostOutStreamer::AddLongLongVector(ChVector<long long> data_in){
  m_long_long_stream_data.push_back(data_in.x());
  m_long_long_stream_data.push_back(data_in.y());
  m_long_long_stream_data.push_back(data_in.z());
} 


void ChBoostOutStreamer::AddChronoVehicleInfo(ChronoVehicleInfo data_in){
  m_vehicle_data.push_back(data_in);
}

void ChBoostOutStreamer::Synchronize() {
  if(m_stream_data.size()!=0){
    boost::system::error_code err;
    auto sent = m_socket->send_to(
    boost::asio::buffer(m_stream_data.data(),
                            sizeof(float) * m_stream_data.size()),
        *m_remote_endpoint, 0, err);
    m_stream_data.clear();
  }else if(m_long_long_stream_data.size()!=0){
      boost::system::error_code err;
      auto sent = m_socket->send_to(
          boost::asio::buffer(m_long_long_stream_data.data(),
                              sizeof(long long) * m_long_long_stream_data.size()),
          *m_remote_endpoint, 0, err);
      m_long_long_stream_data.clear();
  }else{
    for(int i = 0; i < m_vehicle_data.size(); i++){
      boost::system::error_code err;
        auto sent = m_socket->send_to(
            boost::asio::buffer(&m_vehicle_data[i],
                                sizeof(ChronoVehicleInfo)),
            *m_remote_endpoint, 0, err);
    }
    m_vehicle_data.clear();
  }

}

} // namespace hil
} // namespace chrono