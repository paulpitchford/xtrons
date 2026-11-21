<img src="assets/icon.svg" width="64" height="64" alt="">

# Xtrons

Reads fault codes from a Mercedes' control units through an XTRONS (ELM327) Bluetooth OBD2 adapter, from a laptop. Written to find out why the heater blower on a 2016 SLK (R172) ignores its controls.

Generic OBD apps only reach the engine. `slkdiag.py` also asks the other units (climate control, ignition switch, body modules) on Mercedes' own diagnostic addresses.

**Status:** run against a 2016 SLK 250 CDI from a Mac through an XTRONS adapter on 21 November 2026; it found 13 control units and read their fault codes. Also tested against the simulated adapter in `tests/`.

It needs Python 3.9 or newer and nothing else.

## At the car

Plug the adapter into the OBD port and turn the ignition on (key position 2). The engine can stay off, but keep sessions short so the battery is not drained. Then follow the steps for your computer.

### On a Mac

1. Get the code and check Python is there (macOS offers to install it the first time):

   ```
   git clone https://github.com/paulpitchford/xtrons.git
   cd xtrons
   python3 --version
   ```

2. Pair the adapter: System Settings > Bluetooth, wait for a device named OBDII and click Connect. If it asks for a passcode, try 1234, then 0000, then 6789. If pairing is refused, unplug the adapter, plug it back in and try again straight away. The Mac may show the adapter as not connected until the tool opens it.
3. Find the serial port the Mac created for it (look for the one with OBD in its name):

   ```
   ls /dev/cu.*
   ```

4. Run the check with that port:

   ```
   python3 slkdiag.py --port /dev/cu.OBDII
   ```

The adapter only answers on the first time the port is opened after pairing. Before every further run, remove OBDII in Bluetooth settings and pair it again, then run the tool straight away. Opening the port with anything else first (a terminal, a test script) uses up that one chance.

### On Linux

1. Pair the adapter once:

   ```
   bluetoothctl
   scan on                      # wait for a device named OBDII and note its address
   pair AA:BB:CC:DD:EE:FF       # PIN: try 1234, then 0000
   trust AA:BB:CC:DD:EE:FF
   scan off
   quit
   ```

2. Run the check:

   ```
   python3 slkdiag.py AA:BB:CC:DD:EE:FF
   ```

### If no climate unit shows up

Add `--wide` to try every remaining address (a minute or two longer). While it listens to the whole range, a unit that transmits without pause can swamp the adapter; the sweep then names that address, ignores it, and carries on hearing only the half of the range it is not in.

```
python3 slkdiag.py --port /dev/cu.OBDII --wide
```

Each run writes two files to `logs/`: a `.json` with the results and a `.log` with every exchange with the adapter. Keep both; the log is what explains a run that went wrong.

## Options

| Option | Effect |
| --- | --- |
| `--port PATH` | Serial port of a paired adapter, e.g. `/dev/cu.OBDII` on a Mac |
| `--wide` | Also try every address from 600 to 7FF |
| `--probe-only` | Only report which features the adapter supports; sends nothing to the car |
| `--no-listen` | Skip the two-second check for background traffic |
| `--mask HEX` | Fault status mask, default `0D` (current, pending, stored) |
| `--channel N` | Bluetooth serial channel on Linux, default 1 |
| `--tcp HOST:PORT` | Connect to a Wi-Fi adapter or the simulator instead of Bluetooth |

## What it sends

Only read requests: "tester present", "read fault codes" and "read identification". Anything else is refused in code before it reaches the adapter (`READ_ONLY_SERVICES`), so it cannot clear codes, change coding or run actuator tests. It does not transmit on any address it hears another unit using.

## Reading the output

Fault codes are shown raw (`912F87`), the way Mercedes tools and forums write them, and in letter form (`B112F-87`). A few codes carry a hint taken from forum reports on related models; treat those as leads, not definitions.

If a unit holds more codes than were received, the adapter could not keep a long reply going. That is a limit of cheap clones; a better adapter is the fix.

## Without a car

```
python3 tests/fake_elm.py                 # prints the port of a pretend adapter
python3 slkdiag.py --tcp 127.0.0.1:PORT
python3 -m unittest discover -s tests
```

Created in [T3 Code](https://t3.codes).
