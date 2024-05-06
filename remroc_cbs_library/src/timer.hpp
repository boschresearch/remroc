
// This source code is from libMultiRobotPlanning
//   (https://github.com/whoenig/libMultiRobotPlanning)
// Copyright (c) 2017 whoenig
// This source code is licensed under the MIT license found in the
// 3rd-party-licenses.txt file in the root directory of this source tree.
#pragma once

#include <chrono>
#include <iostream>

class Timer {
 public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()),
        end_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

  void stop() { end_ = std::chrono::high_resolution_clock::now(); }

  double elapsedSeconds() const {
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
        end_ - start_);
    return timeSpan.count();
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};

class ScopedTimer : public Timer {
 public:
  ScopedTimer() {}

  ~ScopedTimer() {
    stop();
    std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
  }
};
