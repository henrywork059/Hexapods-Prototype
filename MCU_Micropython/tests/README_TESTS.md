# MCU MicroPython Tests

This folder contains an on-device test harness for the hexapod MicroPython code.

## Quick start (mpremote)

1. Copy the project to the board (or at least the `MCU_Micropython/` folder).
2. From your host machine, run:

```sh
mpremote connect auto fs cp -r MCU_Micropython/ :
mpremote connect auto repl
```

Then at the REPL:

```python
import tests.run_all
tests.run_all.main()
```

## Generic REPL usage

If you are already in a REPL, change to the tests folder and run:

```python
import tests.run_all as run_all
run_all.main()
```

## Hardware tests

Hardware tests are **disabled by default**. To enable:

1. Edit `tests/run_all.py`:
   - Set `RUN_HARDWARE_TESTS = True`
   - Set `SAFE_SERVO_ENABLE = True` if you want servo movement tests
2. When prompted, type `YES` to confirm servo movement.

If you do not confirm, hardware tests are skipped and the run continues.
