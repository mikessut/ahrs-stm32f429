
# Getting started

* Uses GNU ARM toolchain compiler available from
  [ARM](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
  - Loaded in: `C:\Program Files (x86)\GNU Tools ARM Embedded\8 2019-q3-update\bin`
* Uses [Eigen](http://eigen.tuxfamily.org) c++ template library.
* `make` from: `"C:\MinGW\bin\mingw32-make.exe"`
* ST-LINK for linux (open source version) https://github.com/stlink-org/stlink

Setup paths:
```
set PATH=c:\Program Files (x86)\GNU Tools ARM Embedded\8 2019-q3-update\bin;%PATH%
```

Make: `c:\MinGw\bin\mingw32-make.exe`


# Mixing C and C++

https://isocpp.org/wiki/faq/mixing-c-and-cpp#overview-mixing-langs

Attempt here is to just use g++ for compiling the c files.

# Running through VMWare

## CANable

https://canable.io/

Load driver: `sudo slcand -o -c -s4 /dev/ttyACM0 can0`
Activate: `sudo ifconfig can0 up`

Run gateway from: `~/FIX-Gateway`.  
   `fixgw.py` is gateway server (seems to need restart when AHRS board is restarted?)
   `fixgwc.py` is client
pyEfis.py is run from: `~/pyEfis`
