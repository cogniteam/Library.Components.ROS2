// Copyright 2020, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_OUSTER__OS1__OS1_UTIL_HPP_
#define ROS2_OUSTER__OS1__OS1_UTIL_HPP_


#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>

#include "ros2_ouster/OS1/OS1_packet.hpp"

namespace OS1
{
// added by zyl
inline std::vector<double> make_sin_lut()
{
  std::vector<double> sin_array = std::vector<double>(36000, 0);

  for (int icol = 0; icol < 36000; icol++) {
    double angle = 2.0 * M_PI * icol / 36000;    
    sin_array[icol] = std::sin(angle);
  }
  return sin_array;
}

inline std::vector<double> make_cos_lut()
{
  std::vector<double> cos_array = std::vector<double>(36000, 0);

  for (int icol = 0; icol < 36000; icol++) {
    double angle = 2.0 * M_PI * icol / 36000;    
    cos_array[icol] = std::cos(angle);
  }
  return cos_array;
}

template<typename iterator_type, typename F, typename C>
std::function<void(const uint8_t *, iterator_type it, uint64_t)> batch_to_iter2(
  const std::vector<double> & sin_lut,
  const std::vector<double> & cos_lut,
  const std::vector<double> & x_offset_array,
  const std::vector<double> & y_offset_array,
  const std::vector<double> & ah_offset_array,
  const std::vector<double> & av_offset_array,
  uint64_t  &id_frame,
  uint32_t  &id_col,
  int32_t  &azimuth_last,
  int64_t &ts_last,
  uint32_t  &realwidth,
  const typename iterator_type::value_type & empty, C && c, F && f)
{
  //int64_t scan_ts{-1L};

  uint16_t azimuth(0);
  uint16_t distance(0);
  uint8_t intensity(0);
  std::vector<uint16_t> azimuth_array(12,0);
  double div_azimuth(0L);
  uint32_t azimuth_ring(0);

  std::cout << "batch_to_iter2 start" << std::endl;

  return [ = ](const uint8_t * packet_buf, iterator_type it,
           uint64_t override_ts) mutable {
           // 1. get azimuth offset

           for (int icol = 0; icol < 12; icol++) {
             // struct or memcpy?
             memcpy(&azimuth, packet_buf + 100 * icol + 2, sizeof(uint16_t));
             azimuth_array[icol] = azimuth;
           }

           if (azimuth_array[11] >= azimuth_array[0]){
             div_azimuth = (azimuth_array[11] - azimuth_array[0]);
             div_azimuth/=(11*32);
           }
           else{
             div_azimuth = (azimuth_array[11]+36000 - azimuth_array[0]);
             div_azimuth/=(11*32);
           }
           // 2.get timestamp
           uint32_t ts(0);
           memcpy(&ts, packet_buf + 1200, sizeof(uint32_t));

           // 3.scan packet
           for (int icol = 0; icol < 12; icol++) {
             azimuth = azimuth_array[icol];
             if(azimuth_last == -1){
               azimuth_last = azimuth;
             }
             else{
               if(std::abs(azimuth_last - azimuth) > 10000){

                azimuth_last = azimuth;
                // split frame and publish
                if (ts_last != -1){
                  realwidth = id_col;
                  std::cout << "split frame and publish: " << std::to_string(id_col) <<std::endl;
                  //f(override_ts == 0 ? ts_last : override_ts);
                  // from us to ns
                  f(ts_last * 1e3,id_col);
                }
                id_frame++;
                id_col =0;
                ts_last = ts;
               }
             }
             azimuth_last = azimuth;
             
             // write to buf             
             // struct or memcpy?
             for (int irow = 0; irow < 32; irow++) {
               memcpy(&distance, packet_buf + 100 * icol + 4 + irow * 3 , sizeof(uint16_t));
               memcpy(&intensity, packet_buf + 100 * icol + 4 + irow * 3 + 2 , sizeof(uint8_t));

               azimuth_ring = azimuth;
               azimuth_ring += (uint32_t)( irow * div_azimuth);
               azimuth_ring +=36000;
               azimuth_ring %=36000;
                
               
               float r = distance * 0.002;  //unit:m,later get from mdata
               uint8_t ring = irow % 16;


               // x= r * cos(av) * sin(ah) + x_offset * cos(ah)
               // y= r * cos(av) * cos(ah) - x_offset * sin(ah)
               // z= r * sin(av) + v_offset
               uint32_t av_offset_uint32 = (uint32_t)(av_offset_array[ring] * 100);
               av_offset_uint32 +=36000;
               av_offset_uint32 %=36000;
               //        xyDistance * sinAzimuth                               - correction->horizontalOffsetCorrection * cosAzimuth;
               float x = r * cos_lut[av_offset_uint32] * sin_lut[azimuth_ring] + x_offset_array[ring] * cos_lut[azimuth_ring] * 0.001;
               float y = r * cos_lut[av_offset_uint32] * cos_lut[azimuth_ring] - x_offset_array[ring] * sin_lut[azimuth_ring] * 0.001;
               float z = r * sin_lut[av_offset_uint32] + y_offset_array[ring] * 0.001;

               it[id_col * 16 + ring] = c(
                 x,
                 y,
                 z,
				 intensity,
                 (ts - ts_last)*1e3,
                 0,
                 ring,
                 id_frame,
                 0,
                 distance * 2);

               if(ring == 15)id_col++;
             }
           }
         };
}

template<typename iterator_type, typename F, typename C>
std::function<void(const uint8_t *, iterator_type it, uint64_t)> batch_to_iter3(
  const std::vector<double> & sin_lut,
  const std::vector<double> & cos_lut,
  const std::vector<double> & x_offset_array,
  const std::vector<double> & y_offset_array,
  const std::vector<double> & ah_offset_array,
  const std::vector<double> & av_offset_array,
  uint64_t  &id_frame,
  uint32_t  &id_col,
  int32_t  &azimuth_last,
  int64_t &ts_last,
  uint32_t  &realwidth,
  const typename iterator_type::value_type & empty, C && c, F && f)
{
  //int64_t scan_ts{-1L};

  uint16_t azimuth(0);
  uint16_t distance(0);
  //uint8_t intensity(0);          // V1.0
  uint16_t intensity(0);           // V2.0
  std::vector<uint16_t> azimuth_array(12,0);
  double div_azimuth(0L);
  uint32_t azimuth_ring(0);

  std::cout << "batch_to_iter3 start" << std::endl;

  return [ = ](const uint8_t * packet_buf, iterator_type it,
           uint64_t override_ts) mutable {

           // 2.get timestamp,offset 28
           uint32_t ts(0);
           memcpy(&ts, packet_buf + 28, sizeof(uint32_t));

           // 3.scan packet
           // 1 x 150
           for (int icol = 0; icol < 150; icol++) {
        	 memcpy(&azimuth, packet_buf + 40 + 8 * icol, sizeof(uint32_t));
             if(azimuth_last == -1){
               azimuth_last = azimuth;
             }
             else{
               if(std::abs(azimuth_last - azimuth) > 10000){

                azimuth_last = azimuth;
                // split frame and publish
                if (ts_last != -1){
                  // 35999-> 0xFFFF,1600,0xFFFF->0,50
                  if (id_col > 200){
                    // here  realwidth is not update later???
                    realwidth = id_col;
                    // std::cout << "split frame and publish: " << std::to_string(id_col) <<std::endl;
                    //f(override_ts == 0 ? ts_last : override_ts);
                    // from ms to ns
                    f(ts_last * 1e6,id_col);
                  }
                }
                id_frame++;
                id_col =0;
                ts_last = ts;
               }
             }
             azimuth_last = azimuth;
             
             // write to buf             
             // struct or memcpy?
             for (int irow = 0; irow < 1; irow++) {
               memcpy(&distance, packet_buf + 40 + 8 * icol + irow * 8 + 2, sizeof(uint16_t));
               memcpy(&intensity, packet_buf + 40 + 8 * icol + irow * 8 + 4 , sizeof(uint16_t));


               if(distance >=0xFFF0) {
            	   distance =0;
            	   intensity =0;
               }

               azimuth_ring = azimuth;
               azimuth_ring +=36000;
               azimuth_ring %=36000;
                
               
               float r = distance * 0.001;  //unit:m,later get from mdata
               uint8_t ring = 0;

               //  Coordinate: top view
               //            sensor(org,pointcloud)   lidar(ros_base)          transform
               //            y <---o                         x                  beta = 180 degree
               //                  |                         |                  alpha =0
               //         alpha    x                   y <---o                  gamma = 0
               //
               //            laserScan                                          alpha =180
               //                  x                                            beta =0
               //                  |                                            gramma = 0
               //                  0 -->y

               float x = r * cos_lut[azimuth_ring];
               float y = r * sin_lut[azimuth_ring];
               float z = 0;

               it[id_col * 1 + ring] = c(
                 x,
                 y,
                 z,
				 intensity,
                 (ts - ts_last) * 1e6,
                 0,
                 ring,
                 id_frame,
                 0,
                 distance);

               id_col++;
             }
           }
         };
}
}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__OS1_UTIL_HPP_
