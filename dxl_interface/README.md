## Description

This library wraps the Dynamixel SDK. It is intended to abstract details of protocol, model specs and provide more functionalities.

For now, only the ModelSpec class is working properly and it's used in _dxl\_robot\_hw_ package.

## Installation

### Dependencies

* Dynamixel SDK (<https://github.com/ROBOTIS-GIT/DynamixelSDK>)

* YAML-CPP (<https://github.com/jbeder/yaml-cpp>)

### Building

In project's root folder run:

'''
mkdir build
cmake ..
make
'''

### Installing

'''
sudo make install
sudo ldconfig
'''

### Checking

Check if you can find the following files/folders:

* /usr/local/lib/**libdxl_interface.so**

* /usr/local/lib/**dxl_interface/**

* /usr/local/share/**dxl_interface/model_specs**

