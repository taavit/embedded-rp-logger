RP2040 based data logger
========================
First attemp with building orientation device collector.
Basically it collects data from lsm303d sensor and stores it to sd card. Storing can be turned on/off on demand. Led indicate whether it's recording or not.

![Setup](assets/logger.jpg?raw=true "RP2040 Pico Setup")

Data collection sample
----------------------
Example data can be found in [DATA_1.CSV](assets/DATA_1.CSV) containing single capture.

![Acceleration plot](assets/plot_acc.png?raw=true "Acceleration plot")

![Magnetic plot](assets/plot_mag.png?raw=true "Magnetic plot")
