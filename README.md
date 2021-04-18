
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

Or, after switching to candlelight FW

`sudo ip link set can0 up type can bitrate 125000`

If performaing debuging and want a virtual can driver

```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

```
canplayer vcan0=can0 -I logs/candump-2020-12-06_183555.log
awk '{if (substr($1, 2, 17) > 1607190440) { print $0}}' candump-2020-12-05_102623.log | canplayer vcan0=can0
```

## Using CAN-FIX-Utility to flash

```
python3 cfutil.py --channel=can0 --firmware-file ~/ahrs-stm32f429/build/ahrs-stm32f429.hex --target-node=0x12 --device-type=0xb4 --device-model=0x01  --device-version=0x01 --node=1
```

## Debug FW

To set status byte:
```
python3 can_set_param.py -c STATUS 0x6
```