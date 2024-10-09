/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>

typedef std::chrono::duration<double> Duration;

class TicToc {
public:
  TicToc() { tic(); }

  void tic() {
    //
    start = std::chrono::system_clock::now();
  }

  // return in seconds
  double toc() {
    end = std::chrono::system_clock::now();
    Duration elapsed_seconds = end - start;
    return elapsed_seconds.count() * Duration::period::num / Duration::period::den;
  }

  void toc(std::string msg) {
    end = std::chrono::system_clock::now();
    Duration elapsed_seconds = end - start;
    std::cout << msg << " "
              << elapsed_seconds.count() * Duration::period::num / Duration::period::den << "s"
              << std::endl;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
