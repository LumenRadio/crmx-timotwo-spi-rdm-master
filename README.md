# Arduino RDM master 
This is an example of how to use the TimoTwo SPI RDM interface in TX mode to implement a wireless RDM controller/master.

This example does not have fault control, and it also does not deal with ACK_TIMER or ACK_OVERFLOW.

## How to use
Connect your Arduino to your TimoTwo's SPI interface, all five signals are required.

The example will automatically start to discover any connected receivers and downstream RDM devices. It will show you the manufacturer label and device model description of the discovered devices.
It will let you trigger a new discover or to identify any of the discovered devices.

## Configuration
Before building, check `spi_rdm_master/config.h` for settings you may want to change — currently just
`RF_PROTOCOL`, which selects CRMX or G4S mode (defaults to CRMX). Edit the value directly, or override
it with a build flag (e.g. `-DRF_PROTOCOL=TIMO_RF_PROTO_G4S`) without touching the file.

## Building
This project includes a `Makefile` that wraps [`arduino-cli`](https://arduino.github.io/arduino-cli/),
which must be installed and available on your `PATH`.

```sh
make compile   # compile the sketch
make upload    # compile and upload (set PORT if needed)
make monitor   # open a serial monitor
make clean     # remove build artifacts
```

`PORT` and `FQBN` can be overridden, e.g. `make PORT=/dev/ttyACM1 upload`.

## Formatting
Source files are formatted with `clang-format`, enforced via a [`pre-commit`](https://pre-commit.com/) hook and checked in CI on every push and pull request.

To format locally before committing, create a virtual environment, activate it, and install `pre-commit` and set up the git hook once:

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install pre-commit
pre-commit install
```

From then on, activate the virtual environment (`source .venv/bin/activate`) in any new shell before using `pre-commit`. `clang-format` runs automatically on `git commit`. To format everything on demand:

```sh
pre-commit run --all-files --show-diff-on-failure --color=always
```
