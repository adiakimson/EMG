# EMG
By Karolina Joachimczyk, Warsaw University of Technology,
Sep 2022 - Jan 2023
# Short description
This code provides configuration for STM32F042K6T6 Nucleo board that reads 3-channel ADC,
configured with DMA, stores the data and then sends them via either serial port or Bluetooth.
# Further development
It is strongly recommend to switch from a Nucleo 32 to a Nucleo 64 board. This solution provides
only basic aplication of an EMG device. The data are being send via Bluetooth Classic module hc-05
to my desktop app:
https://github.com/adiakimson/emg_desktop

Currently working on a more advanced approach using Nucleo 64 board with BLE.
