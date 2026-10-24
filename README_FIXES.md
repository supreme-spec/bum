# RADUN firmware status

## Target confirmed from the connected flight controller

- Board: ACCIF435
- MCU: AT32F435G
- Installed firmware observed over MSP: Betaflight 4.5.2
- Motor protocol configured in Betaflight: DSHOT600
- Betaflight gyro alignment: CW90 plus `gyro_1_align_yaw = 900`
- Receiver was reporting RXLOSS when the status was captured.
- Battery was not connected when the status was captured.

## Safety and implementation status

The custom image is currently a **bench-development build, not flight firmware**.
The DShot output backend in `dshot_real.c` is deliberately disabled and holds all
four known motor pins low. Arming is blocked while the backend reports unavailable.
Do not replace the working Betaflight installation with this image for flight.

The next steps required before motor output can be enabled are:

1. Implement the AT32F435 timer/DMA backend against the MCU reference manual and
   verify the four outputs with a logic analyzer and propellers removed.
2. Validate the DSHOT600 frame timing, motor order, and direction on the actual ESC.
3. Test sensor axes and attitude response on the bench against Betaflight's
   confirmed sensor alignment.
4. Add a supported receiver/MSP path and run receiver-loss/recovery tests.
5. Validate battery voltage scaling against a meter and the actual battery.
6. Complete hardware-in-the-loop and restrained, propeller-free motor tests before
   considering flight.

## Changes currently in the source

- The CRSF parser validates frame length and CRC, and invalidates stale RC data.
- A neutral-stick interlock is required after startup or receiver loss before arming.
- BMI270 configuration upload is no longer followed by a second soft reset.
- Gyro calibration rejects excessive offset or motion during the calibration window.
- Unverified temperature offsets, in-flight RLS gain changes, duplicate predictive
  feed-forward, and model-based current/blade-loss detection are not used by the
  control loop.
- The BMI270 axes use the alignment reported by the connected Betaflight target.
- Host tests cover failsafe/interlock behavior, anti-gravity utility boundaries,
  FFT peak detection, and the disabled motor-output gate.

## Build

The ARM GNU toolchain is required. Build with the repository Makefile, then verify
the generated image before any use. A successful compile or HEX checksum does not
mean the firmware is suitable for flight.
