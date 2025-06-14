/*
 * do_output.h
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UR_DO_OUTPUT_H_
#define UR_DO_OUTPUT_H_

#ifdef ROS_BUILD
#include "rclcpp/rclcpp.hpp"
#endif
#include <string>

void print_debug(std::string inp);
void print_info(std::string inp);
void print_warning(std::string inp);
void print_error(std::string inp);
void print_fatal(std::string inp);


#endif /* UR_DO_OUTPUT_H_ */
