# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS /Users/Kuba/Documents/esp-idf/TEST1/esp-idf-lib/components)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_init)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_server)
