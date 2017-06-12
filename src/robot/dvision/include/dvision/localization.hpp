/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:51:35+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: localization.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:52:44+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include <ros/ros.h>
#include <vector>

namespace dvision {
class Localization
{
  public:
    explicit Localization();
    ~Localization();
    void init();

  private:
};
} // namespace dvision
