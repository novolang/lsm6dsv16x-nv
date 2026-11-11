# Changelog

## 0.1.0

The first release: a driver for ST's LSM6DSV16X six-axis IMU on an I2C or an
SPI bus, from the data sheet DS13510 revision 3 (March 2023).

- `Lsm6dsv16x` (I2C) and `Lsm6dsv16xSpi` (SPI), the types a board's device
  table builds from the part's wiring, implementing devices-nv's
  `Accelerometer`, `Gyroscope`, `Imu`, `Thermometer` and `Identifiable`.
- The bring-up: the user bank selected, WHO_AM_I, the global reset with
  SW_POR and its 30 ms (IF_CFG survives the software reset), block data
  update and the address increment, the I2C block off on SPI, data-ready
  masked while the filters settle, and both sensors at 120 Hz in high
  performance, ±2 g and ±250 dps.
- The accelerometer in milli-g and the gyroscope in milli-degrees per
  second at the steps of Table 3, the ranges (the gyroscope's ±125 and
  ±4000 dps by code), the data-ready flags, and the temperature.
- The operating modes (high performance, high-accuracy ODR, low power with
  2, 4 or 8 samples averaged, and the accelerometer's normal mode), the rate
  codes with the rate sets of Table 20 and the codes each mode runs, and the
  power-down that high-accuracy ODR mode asks for when it is entered or
  left.
- The interrupt routes to INT1 and INT2, and the pins' polarity and output
  type.
- The FIFO in continuous mode with the tags of Table 218, a watermark of 1 to
  255 words, a nine-bit count, and a read of one word per transaction.
- The sensor fusion low-power block's game rotation vector: enabled, set to
  a rate and put in the FIFO through the embedded functions bank, read out
  as a quaternion in millionths from the half-precision numbers of the
  word, with w from the rest as ST's `sflp2q` computes it.
- The accelerometer's and the gyroscope's self-tests by the procedure of
  ST's `lsm6dsv16x_self_test` example, with the limits of Table 3.
- The device program `examples/imu_reader.nv`, a reading loop. Built for the
  nRF52840 DK it is 15,880 B of flash and for the nRF54LM20 DK 17,344 B.
  `tests/device_build.sh` builds it for both and runs the emulated board's
  image.
- Not verified on hardware: no board on the bench carries the chip. A reply
  from an LSM6DSV16X over the Nordic boards' TWIM and SPIM buses is NOT
  VERIFIED until the driver rig.
- Not implemented: Qvar and the analog hub, the machine learning core and
  the finite state machine, the gravity vector and the gyroscope bias of the
  fusion block, the pedometer and the event detectors, the FIFO's
  compression and timestamp, the OIS and EIS channels, the sensor hub, the
  filters, three-wire SPI, I3C, and the async form of the traits.

### Known

- The data sheet of the LSM6DSV16X is DS13510; a document numbered DS13915
  was not found. ST's server refused the request for the data sheet, which
  was read from SparkFun's copy of revision 3. ST's application note AN5763
  was not read.
- The data sheet says the X, Y and Z components of the quaternion are stored in
  the FIFO (section 1) and does not describe their encoding in the text
  read. The half-precision format and the computation of w are from ST's
  driver and its `lsm6dsv16x_sensor_fusion` example.
- The gyroscope's reserved range codes are read as ±250 dps.
