# Three-motor stepper controller

The Arduino firmware drives three independent stepper drivers. The Tkinter app
can control each motor from its own tab or send coordinated commands to all
three in one serial message.

## Wiring

Connect each driver to the board as follows:

| Motor | PUL+ | PUL- | DIR+ | DIR- |
|---|---:|---:|---:|---:|
| A | 2 | 3 | 4 | 5 |
| B | 6 | 7 | 8 | 9 |
| C | 10 | 11 | 12 | 13 |

These are Arduino digital pin numbers. Upload `stepper_fw/stepper_fw.ino` to a
board that has digital pins 2–13 available; the USB serial connection uses
pins 0 and 1. Connect the driver signal grounds and motor power according to
the driver and motor documentation. Do not power motors from the Mac or the
Arduino's USB connection.

## Run on macOS

1. Install Python 3. If `python3 -m tkinter` does not open a small test window,
   install a Python distribution that includes Tk (the python.org macOS
   installer includes it).
2. From this folder, install the serial dependency:

   ```sh
   python3 -m pip install -r requirements.txt
   ```

3. Upload the firmware using the Arduino IDE. Select the matching board and
   serial port.
4. Start the control app:

   ```sh
   python3 stepper_gui.py
   ```

5. Select the board's `/dev/cu.*` serial port and connect. If the port does not
   appear, click the refresh button after connecting the board.

## Operating modes

- Each motor tab controls its motor independently (asynchronous operation).
- **Move all to each tab's target** sends the three different position targets
  together.
- **Broadcast target to all** sends one shared position to all motors.
- **Rotate all at each tab's speed** starts constant rotation using each tab's
  selected speed.
- **STOP ALL** decelerates all three motors.

The app's default steps/rev is 1600; set it to match the configured driver
microstep resolution. The firmware runs all three control loops on the Arduino,
so motor stepping does not depend on the Mac GUI's refresh rate.
