# Install script for directory: /home/hwpark/capstone2/px4-firmware/src/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/airspeed/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/avoidance/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/battery/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/bezier/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/cdev/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/circuit_breaker/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/collision_prevention/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/component_information/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/controllib/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/conversion/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/crypto/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/drivers/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/field_sensor_bias_estimator/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/geo/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/hysteresis/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/l1/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/landing_slope/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/led/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/matrix/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/mathlib/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/mixer/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/mixer_module/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/motion_planning/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/output_limit/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/perf/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/pid/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/pid_design/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/pwm/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/rc/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/sensor_calibration/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/slew_rate/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/systemlib/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/system_identification/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/tecs/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/terrain_estimation/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/tunes/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/version/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/weather_vane/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/wind_estimator/cmake_install.cmake")
  include("/home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default/src/lib/world_magnetic_model/cmake_install.cmake")

endif()

