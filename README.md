# lsm6dsv16x-nv

A driver for ST's LSM6DSV16X, a six-axis inertial module with a three-axis
accelerometer, a three-axis gyroscope and a temperature sensor, on an I2C or
an SPI bus, specified in ST's data sheet "LSM6DSV16X: 6-axis IMU with
embedded sensor fusion, AI, Qvar for high-end applications", DS13510 Rev 3,
March 2023
([st.com/en/mems-and-sensors/lsm6dsv16x](https://www.st.com/en/mems-and-sensors/lsm6dsv16x.html)).
The driver implements the `Accelerometer`, `Gyroscope`, `Imu`,
`Thermometer` and `Identifiable` traits of
[devices-nv](https://novo-lang.org/packages/devices-nv). It reads the
accelerometer in milli-g and the gyroscope in milli-degrees per second. It
also reads the game rotation vector of the part's sensor fusion low-power
block (SFLP), the attitude of the device as a quaternion, from the FIFO.

**Hardware: NOT VERIFIED.** No board on the bench carries an LSM6DSV16X
yet. The device program below builds for the nRF52840 DK and the
nRF54LM20 DK, and the driver's functions run over a double of the part that
follows the data sheet's register map and bus protocols. A reading from a
part on a board has not been taken, and the program has not been flashed.

## The device program

`examples/imu_reader.nv` reads an LSM6DSV16X on a breakout board ten times a
second and prints the acceleration in milli-g, the rate of turn in
milli-degrees per second and the temperature in thousandths of a degree
Celsius over the board's UART. The breakout's SDA and SCL go to the board's
I2C pins, its CS pin to the supply, which selects I2C, and its SDO/SA0 pin to
ground, which puts the part at 0x6A. Its INT1 pin is left low or open while
the part powers up. With the board flat and still, `acc` shows about 1000 on
its third axis:

```
acc mg 12 -4 1003  gyr mdps 8 -15 3  temp mC 26125
```

```novo norun:needs-pkg
use embedded_hal.{ BoardI2c, BoardGpio }
use dev_imu.{ Imu }
use dev_bus
use lsm6dsv16x.{ Lsm6dsv16x }

@tier(embedded)
fn main() [hw]
    bsp.board.init()
    // The part answers at 0x6A on the board's first I2C bus (SDO/SA0 low).
    let imu = Lsm6dsv16x.attach(BoardI2c { id: 0 }, BoardGpio { id: 0 },
                                dev_bus.wiring_i2c(0, 0x6A))
    // Checks WHO_AM_I, resets the part and starts both sensors.
    if not imu.begin()
        hal.uart.write("lsm6dsv16x did not start\n")
    while true
        match imu.imu_read()
            Some(s) =>
                // s.accel.z is close to 1000 with the board flat.
                hal.uart.write(str.from_int(s.accel.z))
                hal.uart.write("\n")
            None => hal.uart.write("no answer\n")
        hal.timer.delay_ms(100)
```

The full program, with the gyroscope and the temperature, is
`examples/imu_reader.nv`. A board package that depends on this package
builds it as its `main`. `tests/device_build.sh` does so from a scratch
package for the two boards below and for the emulated board, where it runs
the image and checks that the program reports that the part did not start:

```sh
bash tests/device_build.sh
```

| Image | Flash | Of which program | Of which tables and data | RAM (`.data` and `.bss`) |
|---|---|---|---|---|
| `nrf52840-dk` | 15,880 B | 5,310 B | 822 B | 3,236 B |
| `nrf54lm20-dk` | 17,344 B | 5,754 B | 1,774 B | 3,236 B |

The numbers are from the build's memory report. Flash also holds the
runtime and the compiler's support library. The nRF52840 DK's I2C bus is
TWIM0 on SDA P0.26 and SCL P0.27, and the nRF54LM20 DK's is TWIM22 on SDA
P1.03 and SCL P1.04 (`docs/hal/README.md` in the novo-lang repository,
"The Nordic DKs' buses"), so both images carry the reading path. A reply
from an LSM6DSV16X over those buses is NOT VERIFIED until the driver rig.
`BoardI2c` reaches the bus that the board's package names with
`@hal(i2c)`; on a board that names none, the bus is absent hardware, every
transfer is refused, and the program prints `lsm6dsv16x did not start`.

### The effects

Every function of the driver that reaches the bus declares `[hw]`, and
nothing else. The driver performs no input or output of its own and
allocates nothing of its own: every buffer is a fixed-size `Vec` in the
caller's frame. The bus functions of devices-nv build one short frame per
transaction inside a `system` block and release it when the call returns.
The waits go through the bus's `delay_us`, which is `[hw]` too. The package
checks at every tier, `embedded` included.

## What it is

The LSM6DSV16X has a 16-bit three-axis accelerometer, a 16-bit three-axis
gyroscope and a temperature sensor. The accelerometer measures within a full
scale of ±2, ±4, ±8 or ±16 g (1 g is the acceleration of gravity, 9.81 m/s²),
the gyroscope within ±125, ±250, ±500, ±1000, ±2000 or ±4000 degrees per
second (dps). Each axis is a signed 16-bit count, low byte first. The step of
a count is not the full scale over 2^15 (Table 3).

| Full scale | ±2 g | ±4 g | ±8 g | ±16 g |
|---|---|---|---|---|
| Accelerometer step | 0.061 mg | 0.122 mg | 0.244 mg | 0.488 mg |

| Full scale | ±125 dps | ±250 dps | ±500 dps | ±1000 dps | ±2000 dps | ±4000 dps |
|---|---|---|---|---|---|---|
| Gyroscope step | 4.375 mdps | 8.75 mdps | 17.5 mdps | 35 mdps | 70 mdps | 140 mdps |

`GyroRange` has all six: `GyroRange125dps` and `GyroRange4000dps` are the
codes 0 and 12. `GyroRange15dps`, `GyroRange31dps` and `GyroRange62dps` are
below the part's narrowest range and are refused, nothing written. The
readings are converted at the step of the range set. The temperature is `OUT_TEMP / 256 + 25` degrees (Table
6).

Each sensor has its own operating mode and rate. The accelerometer's
operating modes (§ 6.2) are high performance, which runs every rate from 7.5
Hz, normal mode, 7.5 Hz to 1.92 kHz, three low-power modes at 1.875 Hz and 15
to 240 Hz, and high-accuracy ODR mode from 15 Hz. The gyroscope has high
performance, low power at 7.5 to 240 Hz, and high-accuracy ODR mode (§ 6.4).

| Code | 0001 | 0010 | 0011 | 0100 | 0101 | 0110 | 0111 | 1000 | 1001 | 1010 | 1011 | 1100 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Rate (Hz) | 1.875 | 7.5 | 15 | 30 | 60 | 120 | 240 | 480 | 960 | 1920 | 3840 | 7680 |

The gyroscope has no code 0001. High-accuracy ODR mode lowers the
part-to-part variation of the rate (±1 % with the gyroscope on, Table 3) and
has two other sets of rates for the codes 0011 to 1100, chosen by
HAODR_SEL: 15.625 Hz and its doublings up to 8 kHz, or 12.5 Hz and its
doublings up to 6.4 kHz (Table 20).

The part has two bus interfaces on the same pins. CS high selects I2C.

| | I2C | SPI |
|---|---|---|
| Address | 0x6A, or 0x6B with SDO/SA0 high | chip select low |
| Several registers in one read | the address advances while IF_INC is set | the address advances while IF_INC is set |
| Read | register, repeated start, data | control byte with bit 7 set (R/W), then data, with no dummy byte |
| Write | register, then the data | control byte with bit 7 clear, then one byte |
| Clock | 400 kHz asked, 1 MHz at most | 4 MHz asked, mode 3 (mode 0 or 3), 10 MHz at most |

IF_INC, in CTRL3, is set after a reset. IF_CFG is not reset by the software
reset (SW_RESET), so `begin` resets the part with SW_POR, the global reset.

## Install

```sh
novo pkg add lsm6dsv16x-nv
```

## What the package contains

| Module | What is in it |
|---|---|
| `lsm6dsv16x` | The types `Lsm6dsv16x` (I2C) and `Lsm6dsv16xSpi` (SPI), `Lsm6dsv16xMode`, `Lsm6dsv16xQuat`, the trait `Lsm6dsv16xPort`, the bus framing, the register access and the embedded functions bank, the conversions, the rates and operating modes, the FIFO, the game rotation vector, the self-tests and the bring-up |

## How to choose an entry point

A program on a board uses `Lsm6dsv16x` for a part on I2C and
`Lsm6dsv16xSpi` for a part on SPI. Each is built by `attach` with the
board's bus, its GPIO and the wiring, as a board's device table builds it.
Both implement `Accelerometer`, `Gyroscope`, `Imu`, `Thermometer` and
`Identifiable`. The part has a data-ready flag for each sensor, so `ImuControl`
is not implemented.

| Method | What it does |
|---|---|
| `attach(bus, pins, w)` | the part at `w.addr` (I2C) or with the chip select `w.cs` (SPI); nothing is sent |
| `begin()` | brings the part up, see below |
| `accel_read()`, `gyro_read()`, `imu_read()`, `imu_read_temp()` | `Accelerometer`, `Gyroscope`, `Imu`: milli-g, milli-degrees per second, and both from one burst of twelve registers, the second form with the die temperature from one burst of fourteen |
| `accel_ready()`, `gyro_ready()` | the new-data flags of STATUS_REG |
| `accel_range()`, `accel_set_range(r)`, `gyro_range()`, `gyro_set_range(r)` | the full scales, changed without touching the other bits of the register; the gyroscope's ±125 to ±4000 dps |
| `accel_rate_hz()`, `accel_set_rate_hz(hz)`, `gyro_rate_hz()`, `gyro_set_rate_hz(hz)` | the rates in whole hertz, and the slowest rate at or above `hz` that the sensor's mode runs; 0 powers the sensor down |
| `accel_rate_mhz()`, `accel_set_rate_mhz(mhz)`, `gyro_rate_mhz()`, `set_gyro_rate_mhz(mhz)` | the same in thousandths of a hertz, which has 1.875 Hz and the high-accuracy rates exactly |
| `accel_mode()`, `accel_set_mode(m)` | `AccelLowPower` for the low-power modes, `AccelNormal` for normal mode, `AccelHighResolution` for high performance and high-accuracy ODR |
| `set_accel_lp(mean)`, `gyro_mode()`, `set_gyro_mode(m)`, `set_ha_series(sel)` | the number of samples the accelerometer's low-power mode averages (2, 4 or 8), the gyroscope's operating mode, and the rate set of high-accuracy ODR mode; the module's `accel_mode_read` and `accel_mode_write` take the part's own four modes |
| `temp_read()` | `Thermometer`: the die temperature |
| `device_id()` | `Identifiable`: WHO_AM_I, 0x70 |
| `accel_fifo_config(mode, n)`, `accel_fifo_watermark_reached()`, `fifo_end()` | turn the FIFO on in continuous mode (`AccelFifoStream`) or in the part's stop-on-full FIFO mode (`AccelFifoStop`) with the watermark at `n` words, 1 to 32, whether FIFO_WTM_IA is set, and the FIFO off |
| `accel_fifo_count()`, `accel_fifo_read(dst)`, `fifo_take_imu(accel, gyro)`, `fifo_take_quat(dst)` | the FIFO: the word count, the accelerometer's words, both sensors', and the game rotation vectors |
| `imu_fusion_enable(on)`, `imu_orientation()` | `Imu`: the sensor fusion low-power block on (with its words in the FIFO, and the FIFO started when it is in bypass) and off, and the newest game rotation vector as a Q30 quaternion; reading it empties the FIFO |
| `set_fusion_rate(ctrl)`, `set_fusion_fifo(on)`, `reset_fusion()` | the sensor fusion low-power block: its rate, its place in the FIFO, a restart |
| `route_int1(mask)`, `route_int2(mask)`, `configure_pins(active_low, open_drain)` | the interrupt pins |
| `accel_test(negative)`, `gyro_test(negative)` | the change in milli-g and in milli-degrees per second that each self-test makes |
| `reg(r)`, `set_reg(r, v)`, `emb_reg(r)`, `set_emb_reg(r, mask, v)` | one register, a write that is read back, and a register of the embedded functions bank |

`begin` puts the register bank to the user bank, checks WHO_AM_I, resets the
part with SW_POR and waits 30 ms, sets block data update and the address
increment, turns the I2C block off on SPI, masks the data-ready while the
filters settle, puts the full-scale mode at ±2 g and ±250 dps, puts both
data-ready flags on INT1, and starts both sensors in high-performance mode
at 120 Hz. It then waits the gyroscope's 30 ms turn-on. It answers `false`
for a part that does not answer 0x70 and for a register that does not hold
what was written.

A program that holds its own bus, or a test with a double of the part,
implements `Lsm6dsv16xPort` over its bus and calls the module's functions,
which take the port: `lsm6dsv16x.imu_read_of(port)`,
`lsm6dsv16x.start(port)`. The framing of each bus is `i2c_read_into`,
`i2c_write`, `spi_read_into` and `spi_write`, generic over `I2cBus` and
`SpiBus`.

### The FIFO

The FIFO is an option. It is 1.5 KB of words of a tag byte and six data
bytes, and `begin` leaves it off. `accel_fifo_config` puts it in continuous
mode (`AccelFifoStream`), which overwrites the oldest word when it is full,
or in FIFO mode (`AccelFifoStop`), which stops collecting when it is full.
It batches the accelerometer at its present rate and the gyroscope too when
the gyroscope is running, and puts the watermark on INT1 in place of the
data-ready flags. The watermark is 1 to 32 words through the trait (the
module's `fifo_start` takes up to 255), and `accel_fifo_watermark_reached`
reads FIFO_STATUS2's FIFO_WTM_IA, which the part keeps as a level.
`AccelFifoOff` is `fifo_end`. The part counts words of every kind, nine bits of them, so with
both sensors batched the count is the accelerometer's and the gyroscope's
together, and the words alternate. Compression is off.

The tag says what a word is: 1 the gyroscope, 2 the accelerometer, 0x13 the
game rotation vector. `accel_fifo_read` is the trait's and answers the
accelerometer's words: it takes every word out, because a word can only be
read in order, and drops the others. `fifo_take_imu` fills two lists and
stops, without taking a word, as soon as either is full. `fifo_take_quat`
keeps the game rotation vectors. All convert with the full scales read from
the part before the first word.

### The game rotation vector

The sensor fusion low-power block computes from the accelerometer and the
gyroscope the attitude of the device as a unit quaternion, the game
rotation vector, and writes it to the FIFO as a word tagged 0x13. The six
data bytes are the quaternion's x, y and z as IEEE 754 half-precision
numbers, low byte first; w is not stored and is the square root of one
minus the sum of their squares, as ST's `sflp2q` computes it. The driver
answers an `Lsm6dsv16xQuat` of four integers in millionths, so 1,000,000 is
1.0, from `fifo_take_quat`, and `imu_orientation` answers the newest as the
trait's `ImuQuat` in Q30. The part delivers the vector through the FIFO and
nowhere else, so `imu_orientation` takes every word out of the FIFO, drops
the sensors' words and answers `None` when the FIFO holds no vector; a
program that also reads the sensors' words uses `fifo_take_quat`.
`imu_fusion_enable(true)` enables the block, puts its words in the FIFO and
starts the FIFO in continuous mode when it is in bypass.

```novo norun:fragment
// The block runs at 60 Hz, so the sensors run at 60 Hz or more.
let _ = imu.accel_set_rate_mhz(60000)
let _ = imu.set_gyro_rate_mhz(60000)
let _ = imu.set_fusion_rate(2)       // 15, 30, 60, 120, 240, 480 Hz
let _ = imu.set_fusion_fifo(true)
let _ = imu.imu_fusion_enable(true)
let _ = imu.accel_fifo_config(AccelFifoMode.AccelFifoStream, 16)   // words
// Later, when the watermark pin falls or the count is not 0:
var q: Vec[Lsm6dsv16xQuat; 32] = Vec.new()
let n = imu.fifo_take_quat(var q)
```

The block is enabled and configured in the embedded functions bank, which
FUNC_CFG_ACCESS selects. Each function that visits the bank puts the user
bank back, also when the work failed, and answers `false` if either the
work or the restore failed.

## The rules a user needs

1. Leave INT1 low or open while the part powers up: with INT1 tied to the
   supply the part comes up on I3C only (§ 7).
2. `begin` has to run before a reading: both sensors are in power-down after
   a reset. It waits the gyroscope's 30 ms turn-on; `gyro_set_rate_hz`
   waits it too when the gyroscope was off.
3. A reading is converted at the full scale read in the transaction before
   it. After a change of range or rate the filters settle for a few
   samples, which the data-ready line is masked for (DRDY_MASK).
4. `accel_set_rate_hz(0)` and `gyro_set_rate_hz(0)` power a sensor down. The
   temperature sensor is refreshed only while a motion sensor is on.
5. The accelerometer and the gyroscope each have their own rate.
   `imu_read` pairs whatever the two registers hold, so for samples that
   belong together the two rates are set equal and the pair is read when
   both flags are set.
6. A rate is taken in the rate set of the sensor's mode. In low-power mode
   it is one of 1.875 Hz and 15 to 240 Hz (the gyroscope: 7.5 to 240 Hz); in
   normal mode up to 1.92 kHz; in high-accuracy ODR mode from 15 Hz, in the
   set HAODR_SEL names. The rates in whole hertz are rounded to the nearest,
   so 1.875 Hz reads as 2 and 7.5 Hz as 8. Use the thousandths of a hertz for
   the exact rate.
7. High-accuracy ODR mode has to be entered and left with the sensor in
   power-down, and when one sensor is in it the other has to be in it too
   (§ 6.5). `accel_set_mode` and `set_gyro_mode` power the sensor down for
   such a change and write the rate back, moved to the nearest rate the new
   mode runs; the other sensor's mode is the program's. HAODR_SEL is chosen
   with `set_ha_series` while the sensors are in power-down.
8. The FIFO is configured from bypass: `accel_fifo_config` writes bypass
   first, then the watermark, the batch rates, the interrupt route, and the
   mode last. Batching is at the rates the sensors run at, so a change of
   rate after `accel_fifo_config` leaves the batch rate where it was.
9. The game rotation vector needs the accelerometer and the gyroscope on at
   the rate of the block or faster. `reset_fusion` starts the block over.
   The block's calibration takes 0.8 s and the orientation settles in 0.7 s
   (Table 1).
10. The self-tests need the board still. The driver follows the procedure of
    ST's `lsm6dsv16x_self_test` example: the accelerometer at 60 Hz and
    ±4 g, the gyroscope at 240 Hz and ±2000 dps, the output left to settle
    for 100 ms, one sample passed, five samples averaged with the self-test
    off and five with it on. Table 3 gives the accelerometer's change as 50
    to 1700 mg whatever the full scale, and the gyroscope's as 150 to 700
    dps at ±2000 dps. `accel_self_test_within` and `gyro_self_test_within`
    compare a change with them by magnitude. The sign of the change follows
    the `negative` argument. The rate, the mode and the range the sensor had
    are put back.
11. SPI mode 0 and mode 3 both work (§ 5.1), and the driver asks for mode 3.
    `begin` sets I2C_I3C_disable in IF_CFG on SPI, which turns the I2C and
    I3C blocks off.
12. `device_id` answers `None` for a bus that no part drives: an SPI bus
    reads 0xFF, or 0x00 when held low.

## What is not included

- Qvar, the electric-charge-variation input, and the analog hub. NOT
  IMPLEMENTED. They are not compatible with high-accuracy ODR mode.
- The machine learning core, its filters and features in the FIFO, and the
  finite state machine, with the adaptive self-configuration. NOT
  IMPLEMENTED. The embedded bank is reachable through `emb_reg` and
  `set_emb_reg`.
- The gravity vector and the gyroscope bias of the sensor fusion block, in
  the FIFO as tags 0x17 and 0x16, and the initial bias (`SFLP_GAME_GBIAS`).
  NOT IMPLEMENTED; the words are taken out and dropped.
- The pedometer, tilt detection and significant motion, the free-fall,
  wake-up, tap and 6D/4D event detectors, and activity and inactivity. NOT
  IMPLEMENTED; they are configured through the registers `reg` reaches.
- The FIFO's compression, its counter of batch events, its timestamp and
  temperature batching, the sensor hub's external sensors in the FIFO, and
  the dual-channel accelerometer. NOT IMPLEMENTED.
- The OIS and EIS channels and the auxiliary SPI, the sensor hub (the I2C
  master for external sensors), ODR-triggered mode, the filters (LPF1,
  LPF2, the high-pass), the user offsets, and the I3C interface. NOT
  IMPLEMENTED.
- The three-wire SPI mode, and the LSM6DSV16BX and the other parts of the
  family.
- The async form of the traits. NOT IMPLEMENTED: devices-nv's
  `AccelerometerAsync`, `GyroscopeAsync` and `ThermometerAsync` are
  `async fn` traits, which `@tier(embedded)` refuses (E4002) and which take
  no `@value` struct, and the board handles are `@value` structs. The part
  completes nothing in the background that a program waits for: it samples
  on its own clock and the program reads the data-ready flag or the line.
  The interrupt line as `AsyncDeviceReady` needs an edge source on
  `BoardGpio` that embedded-hal-nv does not have.

## Where the traits fall short

- `AccelerometerAsync`, `GyroscopeAsync` and `ThermometerAsync` are not
  implemented: `async fn` is refused at the embedded tier, and a `@value`
  driver cannot implement them.
- `AsyncDeviceReady` has no source, because `BoardGpio` has no `AsyncEdge`.
- A trait method gets a plain `self`, so the driver keeps no state between
  calls.
- `ImuControl` is not implemented: the part has a flag and a rate for each
  sensor, and the flags are not cleared by a shared read.
- `Imu.imu_orientation` is a read of the FIFO, since the part delivers the
  vector there and nowhere else, so it empties the FIFO. The game rotation
  vector is the block's only output the driver reads; the gravity vector and
  the gyroscope bias are not.
- `GyroRange15dps`, `GyroRange31dps` and `GyroRange62dps` are refused: the
  narrowest range of the part is ±125 dps.

## Related packages

- [devices-nv](https://novo-lang.org/packages/devices-nv) names the
  traits this driver implements, the wiring record and the register
  helpers it uses.
- [embedded-hal-nv](https://novo-lang.org/packages/embedded-hal-nv) names
  the `I2cBus`, `SpiBus` and `GpioOut` traits and the board handles.
- lsm6dso-nv drives ST's LSM6DSO and LSM6DSOX, which have the machine
  learning core and the finite state machine but not the sensor fusion
  block.
- bmi270-nv and icm42688-nv drive six-axis IMUs from Bosch Sensortec and
  TDK InvenSense, with the same traits.

## Tests

`novo test` runs the module over a double of the part that answers on an
I2C bus and on an SPI bus, with the embedded functions bank: the control
byte, the address increment and what happens when IF_INC is clear, WHO_AM_I,
every conversion at every range of both sensors, the rate codes in all
three rate sets and the codes each mode runs, the operating modes and the
power-down that high-accuracy mode needs, the FIFO with each kind of word,
the half-precision numbers and the quaternion that ST's `sflp2q` computes,
the visits to the embedded bank with the user bank put back also when the
work or the entry failed, the self-tests, the bring-up from stale settings
with each write refused in turn, and the two types over the host's board
handles. The log of bytes sent is compared for each entry point. A second
port refuses a chosen write or read of a chosen register, which reaches the
refusals a read-only register cannot. The suite covers every line of
`src/`, and `tests/coverage.sh` measures it:

```sh
NOVO=bin/novo bash tests/coverage.sh
```

**Hardware: NOT VERIFIED.** See the device program section.

The sources read for this driver, for their fixes and not ported, are the
data sheet above, ST's platform-independent driver `lsm6dsv16x_reg` with its
examples `lsm6dsv16x_self_test` and `lsm6dsv16x_sensor_fusion`, the Rust
crate `lsm6dsv16x-rs`, which wraps ST's driver, and Zephyr's
`drivers/sensor/st/lsm6dsv16x`. ST's application note AN5763 is cited by
number, as Zephyr's driver cites it, and was not read. Embassy and esp-hal
hold no driver for the part. Each fix kept is named in `src/lsm6dsv16x.nv`
where it is kept:

- Zephyr puts the part in the user bank before it reads WHO_AM_I, and resets
  it with SW_POR and a wait of 30 ms; so does `begin`, which also writes
  IF_CFG afterwards, since the software reset leaves it alone.
- ST's driver restores the user bank after each visit to the embedded bank
  and changes single fields of registers that hold bits "that must be 0" or
  "that must be 1"; so do the functions here.
- ST's driver powers a sensor down for the software reset and for the
  reboot, and the data sheet asks for it when high-accuracy ODR mode is
  entered or left; `accel_set_mode` and `set_gyro_mode` do.
- ST's FIFO examples set the FIFO to bypass before they change its
  configuration, and write the mode last; so does `accel_fifo_config`.
- ST's `sflp2q` reads x, y and z as half-precision numbers, scales a vector
  longer than 1 and takes w from the rest; `quat_at` does, in integers.
- ST's `lsm6dsv16x_self_test` example gives the rates, ranges, settling and
  limits the self-tests use.

## Licence

Apache-2.0.
