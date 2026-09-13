# Local Lizard Firmware Setup

Lizard firmware is developed and built in a separate checkout.
This ROS repository intentionally contains no Lizard source tree or symlink under `src/`.

## Checkout

Clone Lizard outside this repository, for example beside it in your workspace:

```bash
git clone https://github.com/Agroecology-Lab/lizard.git ../lizard
cd ../lizard
git checkout 87ab4e198d3af2a274e60292ee204b722f939b5e
git submodule update --init --recursive
```

The revision matches the `LIZARD_COMMIT` used by this repository's Docker image.
Use another revision only when the firmware and ROS integration are intentionally being updated together.

## Build and flash

Follow the Lizard repository's [build instructions](https://github.com/Agroecology-Lab/lizard/blob/87ab4e198d3af2a274e60292ee204b722f939b5e/README.md) from that external checkout.
They build ESP32-S3 firmware with the `espressif/idf:v5.3.1` container.

After a successful build, flash the connected controller from the Lizard checkout:

```bash
sudo ./espresso.py flash --device /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the controller's serial device.
The serial device must not be in use by a ROS driver or monitor while flashing.

Do not copy the Lizard checkout or its generated `build/` directory into this repository.
