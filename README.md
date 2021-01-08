# RSPALSAServer
RSP ALSA Server

This is server code to transfer data from RSP devices to ALSA.
RSP (Radio Spectrum Processor) devices are SDR (Software Defined Radio) devices containing ADCs (Analog-Digital Converters).
This code was created using RSPTCPServer as reference: https://github.com/SDRplay/RSPTCPServer

(c)2021 KK7DV Gary (CrumResearch@gmail.com). Licensed under the GNU GPL V3

## an rsp alsa IP server for the RSP range of SDRPlay SDR

## OPTIONS

```
 -o output ALSA device
 -d RSP device to use (default: 1, first found)
 -P Antenna Port select (0/1/2, default: 0, Port A)
 -T Bias-T enable (default: disabled)
 -N Broadcast Notch enable (default: disabled)
 -R Refclk output enable (default: disabled)
 -f frequency to tune to [Hz]
 -s samplerate in Hz (default: 2048000 Hz)
 -n max number of linked list buffers to keep (default: 500)
 -v Verbose output (debug) enable (default: disabled)
 -E extended mode full RSP bit rate and controls (default: RTL mode)
```

## BUILDING
```
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
```
## NOTES
 - a RSP API version >=2.13 must be installed on the linux server, see [sdrplay linux downloads](https://www.sdrplay.com/downloads/)

## TODO

## HISTORY

## CREDITS
 - [SDRplay](https://github.com/SDRplay/RSPTCPServer)
 - [Open Source Mobile Communications (OSMOCOM)](https://github.com/osmocom/rtl-sdr.git) team for the original rtl_tcp code
 - [Thierry Leconte](https://github.com/TLeconte/airspy_tcp.git) for many ideas that I found in his Airspy port of rtl_tcp
 - [Tony Hoyle](https://github.com/TonyHoyle/sdrplay.git) for the initial idea
 - [Pothosware](https://github.com/pothosware) for the cmake build examples
