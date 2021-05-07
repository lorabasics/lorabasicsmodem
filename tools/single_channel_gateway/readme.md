	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2021 Semtech-Cycleo

Single Channel Gateway
======================

This project is a demonstrator of a single channel gateway based on a SX1268 radio.
It is based on MBED development environment.

# 1. Configure mbed environment

Setup MBED as documented here:

[Mbed Installers](https://os.mbed.com/docs/mbed-os/v6.9/build-tools/install-and-set-up.html)

# 2. Hardware Modification

In order to enable the network interface functional, `NUCLEO_F429ZI` board needs to be modified.
Here is [How to patch the NUCLEO board](https://os.mbed.com/teams/ST/wiki/Nucleo-144pins-ethernet-spi-conflict).

# 3. Download and compile the project

This project needs `mbed-os` to compile, so user needs to download a specific version "5.7.4" of `mbed-os` first.

- Download `mbed-os-5.7.4` from https://github.com/ARMmbed/mbed-os/archive/refs/tags/mbed-os-5.7.4.zip.
- Unzip it to the root folder of single channel gateway project, and rename it to be just **mbed-os**.
  So that folder would contain folders and files as below:
```
  jsmn  mbed-os  PROTOCOL.md  readme.md  src  sx126x-driver  user_manual.pdf
```
- Open file `mbed-os/targets/targets.json` and change **PA_7** to **PB_5** at line `1166`.
- Compile it with below command:
```
mbed compile -c -t GCC_ARM -m NUCLEO_F429ZI --source src --source sx126x-driver --source mbed-os --source jsmn
```

# 4. Gateway Configuration

All parameters that can be adapted to the user needs are located in the `src/userconf.h` file.

## 4.1. Channel frequency configuration

In order to set the channel frequency to be used, one can change the `channel_freq` field of the `gw_conf` variable.
It is by default set to `CHANNEL_FREQ`, but can be changed as needed.

For an automatic setting of the channel frequency, it is possible to fill the `gw_freq_conf`
variable to assign a channel frequency to a particular MAC address.
After the gateway boots, it will get its own MAC address, and check in the `gw_freq_conf`
array if there is a match, and get the channel frequency to be used accordingly.

In order to get your gateway MAC address, you can first plug you gateway on a USB port of a PC,
then wait for the serial logs to show the detected MAC address.

Please note that the gateway must be connected on a local network in order to see the MAC address.

## 4.2. Channel datarate configuration

In order to set the channel datarate to be used, one can change the `sf` and `bw` fields of the `gw_conf` variable.

## 4.3. Antenna switch alternate

The `antenna_switch_alternate` filed of the `gw_conf` variable can be
used to enable or disable switching from one antenna to the other between each
packet received.
It is important to set it to false if only one antenna is used.

## 4.4. LoRaWAN Network Server configuration

In order to set the Network Server UDP parameters to be used,
one can change the `server_address` and `server_port` fields of the `gw_conf` variable.

Please refer to the `PROTOCOL.md` file for more details about the protocol used
between the Gateway and the Network Server.
