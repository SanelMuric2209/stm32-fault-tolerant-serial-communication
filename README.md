# STM32 Fault-Tolerant Serial Communication

A fault-tolerant serial communication link implemented on the STM32H753ZI (NUCLEO-H753ZI) that can survive severe electrical noise on the wire. The project is intentionally called the "3 Board Problem" because making it work as intended requires three separate development boards: one acting as the noise maker and two running this firmware as sender and receiver.

---

## What This Does

The core idea is straightforward: take a serial byte stream from a host PC, relay it across a potentially very noisy physical wire to a second board, and deliver it cleanly on the other side — correcting or detecting any corruption that happens in transit.

The firmware sits in the middle of that path. It bridges a USB serial connection (from a laptop) to a raw UART link that goes out over an actual wire to another board. On the receiving side the roles are mirrored: the second board accepts the noisy wire data and sends the cleaned result back to its own laptop.

### Why Three Boards

The full demo requires:

| Board | Role |
|-------|------|
| Board 1 | Sender — reads characters from a PC terminal and encodes them onto the wire |
| Board 2 | Receiver — decodes the wire data and forwards clean characters to its own PC |
| Board 3 | Noise maker — intentionally injects electrical noise onto the wire between boards 1 and 2 |

Without the noise maker (board 3), the link works trivially over a short wire. The noise maker is what makes the fault-tolerance actually meaningful and testable. The same firmware runs on both board 1 and board 2; the direction of data flow is determined by which UART carries the laptop connection and which carries the wire.

---

## How It Works

### The Wire Encoding Layer

Each byte sent over the wire is transmitted **301 times in a row** (`WIRE_REPETITIONS = 301`). Before each repetition the byte is XORed with a rotating mask and bit-rotated, so every repetition looks slightly different on the wire. This prevents a constant-level noise source from systematically corrupting every copy the same way.

On the receiving side, all 301 copies of each bit position are tallied. The majority value wins — if more than 150 of the 301 copies of a given bit are `1`, that bit is decoded as `1`. This is a hardware majority vote implemented in software. Because 301 is odd, there are no ties. A 50 µs pause is inserted between successive bytes to give the receiver time to keep up.

If any bit position did not vote unanimously (i.e., some copies differed), the byte is marked as "corrected" rather than clean. This is used downstream for status reporting and LED feedback.

### The Frame Format

Bytes are not sent raw; they are wrapped in a **framed protocol** on top of the wire encoding layer. Each frame carries:

```
[ 0x55 | 0xAA | sequence | data | CRC32 (4 bytes) ]
```

- **Sync bytes** (`0x55`, `0xAA`) allow the receiver to find the start of a frame even after corruption or re-synchronisation.
- **Sequence number** is an 8-bit counter that increments with every data byte. The receiver uses this to detect dropped or reordered characters and inserts `?` placeholders for any gaps.
- **Data** is the single application byte being carried.
- **CRC32** is computed over the sequence and data bytes using the standard IEEE 802.3 polynomial. This lets the receiver confirm that the majority-voted result actually makes sense, or discard it if it does not.

Each frame is sent **3 times** (`FRAME_COPIES = 3`) in sequence. The receiver accepts whichever copy first produces a passing CRC.

The framing layer also handles **polarity inversion**: if the CRC check fails with the raw bytes but passes when both `sequence` and `data` are bit-inverted, the frame is still accepted and flagged as corrected. This covers the case where the noise maker flips the entire signal level.

### Reset Frames

At the start of each new line of text, a special **reset frame** is sent before any data frames. Its data byte is `0xC3` and its CRC is XORed with a known constant (`FRAME_RESET_XOR = 0xA5F03C69`). When the receiver sees this it re-synchronises its expected sequence number, which prevents one corrupted message from causing the next one to appear garbled.

### LED Status Indicators

| LED | Meaning |
|-----|---------|
| Green | Firmware is running and ready |
| Yellow | A majority vote was used or polarity was corrected (data recovered) |
| Red | A CRC check failed and a frame copy was discarded |

---

## Hardware Required

- **3x STM32 NUCLEO-H753ZI** development boards (STM32H753ZIT6, LQFP144)
- USB cables for each board (for power and PC serial connection)
- Jumper wires to connect the UART2 TX/RX pins between board 1 and board 2
- A noise injection circuit or a third board programmed to act as a noise maker on that wire

### Pin Connections

The firmware uses two UART peripherals on each board:

| Peripheral | Pins | Purpose |
|------------|------|---------|
| USART3 | PD8 (TX), PD9 (RX) | Connection to the host PC (laptop side) |
| USART2 | PD5 (TX), PD6 (RX) | Connection to the wire between boards |

Both UARTs run at **115200 baud, 8N1**.

Wire the two boards together like this:

```
Board 1 PD5 (USART2 TX)  -->  [noise maker]  -->  Board 2 PD6 (USART2 RX)
Board 1 PD6 (USART2 RX)  <--  [noise maker]  <--  Board 2 PD5 (USART2 TX)
GND  <-->  GND  (common ground between all three boards)
```

The noise maker board sits in-line on those wires and injects electrical interference. Without it the link degrades to a straightforward UART relay.

---

## Software Requirements

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) or a standalone ARM GCC toolchain (`arm-none-eabi-gcc`)
- [CMake](https://cmake.org/) 3.22 or newer
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) (optional, only needed if you want to modify the `.ioc` peripheral configuration)
- STM32CubeProgrammer or OpenOCD for flashing
- STM32Cube FW_H7 V1.13.0 HAL library (referenced by the `.ioc` file; CubeMX will download it automatically)

---

## Building

The project uses CMake with the ARM GCC toolchain. A `CMakePresets.json` is included.

```bash
# Configure
cmake -B build --preset Debug

# Build
cmake --build build
```

The output binary will be at `build/3BoardProblem.elf` (and `.bin`/`.hex` equivalents depending on your toolchain setup).

If you are using STM32CubeIDE, import the project directly — the IDE will handle the build configuration automatically.

---

## Flashing

Using STM32CubeProgrammer CLI:

```bash
STM32_Programmer_CLI -c port=SWD -w build/3BoardProblem.hex -v -rst
```

Or drag the `.bin` file onto the virtual USB drive that appears when the NUCLEO board is connected (mass storage flashing).

---

## Running

1. Flash the same firmware onto both board 1 and board 2.
2. Wire the boards together through the noise maker as described above.
3. Open a serial terminal (115200 baud, 8N1) on the PC connected to each board's USART3 port. On Windows this is the STLink virtual COM port that appears in Device Manager.
4. On startup the board prints:

   ```
   Robust link ready. Characters are printed immediately.
   ```

5. Type text into the terminal on board 1's PC. The characters travel across the noisy wire and appear in the terminal on board 2's PC.
6. Watch the LEDs: yellow flickers when data had to be corrected by majority vote; red flickers when a full frame copy was rejected by the CRC check.

---

## Key Parameters

All tunable constants are defined at the top of [`Core/Src/main.c`](Core/Src/main.c):

| Constant | Default | Description |
|----------|---------|-------------|
| `WIRE_REPETITIONS` | 301 | Number of times each byte is repeated on the wire. Must be odd. |
| `WIRE_PAUSE_US` | 50 | Microsecond pause inserted between wire bytes. |
| `FRAME_COPIES` | 3 | Number of times each frame is sent in sequence. |
| `FRAME_SYNC_1` | `0x55` | First sync byte of every frame. |
| `FRAME_SYNC_2` | `0xAA` | Second sync byte of every frame. |
| `WIRE_MASK_COUNT` | 8 | Number of encoding phases used to vary each repetition. |

Increasing `WIRE_REPETITIONS` makes the link more resilient to noise at the cost of throughput. At 301 repetitions and 115200 baud the raw wire bandwidth is almost entirely consumed by redundancy, leaving an effective data rate of a few hundred bytes per second at most.

---

## Project Structure

```
3BoardProblem.ioc          STM32CubeMX peripheral configuration
CMakeLists.txt             Top-level CMake build file
CMakePresets.json          CMake preset for Debug builds
STM32H753XX_FLASH.ld       Linker script
startup_stm32h753xx.s      Startup assembly file
Core/
  Inc/
    main.h                 Application header
    usart.h                UART peripheral header
    gpio.h                 GPIO header
  Src/
    main.c                 All application logic (encoding, framing, majority vote)
    usart.c                UART peripheral initialisation (USART2 and USART3)
    gpio.c                 GPIO initialisation
    stm32h7xx_it.c         Interrupt handlers
Drivers/                   STM32Cube HAL and BSP drivers
```
