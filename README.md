# I2C Address Translator

[![Ask DeepWiki](https://devin.ai/assets/askdeepwiki.png)](https://deepwiki.com/tejasgolhar2/i2c_translator)

This repository contains Verilog RTL for an I2C address translator. This module acts as an intermediary device, allowing a single I2C master to communicate with multiple slave devices that share the same physical I2C address. It achieves this by presenting unique "virtual" addresses to the master and translating them to the common physical address for the appropriate downstream slave.

The core functionality is encapsulated in two main components:
1.  **Upstream Slave (`upstream_slave.v`):** Faces the main I2C master, listens for commands, and identifies the target virtual slave address.
2.  **Downstream Master (`downstream_master.v`):** Faces the physical slave devices and initiates transactions to the selected slave using its physical address.

This design effectively multiplexes the I2C bus based on virtual addressing, resolving address conflicts on the bus.

## Key Features

*   **Address Translation:** Maps unique virtual I2C addresses (e.g., `0x48`, `0x49`) to a common physical address (`0x48`).
*   **Bus Multiplexing:** Enables a single master to control two slaves with identical addresses by selecting them through different virtual addresses.
*   **Clock Stretching:** The upstream slave module performs clock stretching on the main master's SCL line while the downstream master is communicating with a physical slave, ensuring transaction integrity.
*   **Standard I2C Protocol:** The downstream master generates a standard 400kHz I2C clock from a 100MHz system clock for communication with the physical slaves.
*   **Write Transaction Support:** The design fully supports I2C write sequences, including start, address, data, and stop conditions.

## How It Works

1.  An external I2C master initiates a transaction by sending a `START` condition followed by a virtual slave address (e.g., `0x48` for Slave A or `0x49` for Slave B).
2.  The `upstream_slave` module within the translator captures this address. It begins clock-stretching the external master to pause the transaction.
3.  Based on the virtual address received, the translator logic determines which physical slave (A or B) is the target.
4.  The `downstream_master` module then initiates a new I2C transaction on the corresponding physical bus (`sda_a`/`scl_a` or `sda_b`/`scl_b`). It uses the slaves' common physical address (e.g., `0x48`).
5.  The `downstream_master` sends the sub-address and data bytes, which it has received from the `upstream_slave`.
6.  Once the downstream transaction is complete (indicated by a `STOP` condition or receiving all expected ACKs), the `upstream_slave` releases the clock-stretching.
7.  The transaction with the external master is then completed with an `ACK`, allowing it to send the next byte or a `STOP` condition.

## Repository Structure

```
.
├── 1_Verilog_Modules/
│   ├── downstream_master.v   # I2C master logic to control physical slaves
│   ├── top.v                 # Top-level DUT, connecting upstream/downstream modules
│   └── upstream_slave.v      # I2C slave logic that faces the main master
├── 2_Testbench/
│   ├── i2c_master_str.v      # I2C Master model (part of the testbench driver)
│   ├── i2c_master_top.v      # Testbench driver that initiates transactions
│   ├── i2c_slave_a.v         # Model for physical I2C Slave A
│   ├── i2c_slave_b.v         # Model for physical I2C Slave B
│   └── sim_i2c.v             # Top-level simulation file
├── 3_Simulation_on_EDA_Playground/
│   └── 0_EDAPlayground/
│       └── EDAPlayground Link.txt # Link to the online simulation environment
├── 4_Resource_Report/          # Directory for synthesis resource reports
└── 5_Brief_Documentation/      # Directory for additional documentation
```

## Simulation

A complete testbench is provided to verify the functionality of the translator.

*   **Test Scenario:** The testbench (`i2c_master_top.v`) simulates an I2C master sending a 3-byte write command to virtual address `0x48`. The translator receives this, stalls the master, and forwards the command to the physical slave A (which has a physical address of `0x48`).
*   **Components:**
    *   `sim_i2c.v`: The simulation top-level that instantiates the DUT, the test master, and the two physical slave models.
    *   `i2c_master_top.v`: A wrapper that generates a sample write transaction, simulating a DAC programming sequence.
    *   `i2c_slave_a.v` / `i2c_slave_b.v`: Simple I2C slave models that acknowledge their address and receive data.

### Running the Simulation on EDA Playground

You can run the simulation directly in your browser without any local setup using the provided EDA Playground project.

**[Click here to open the project on EDA Playground](https://edaplayground.com/x/Fynx)**

Once in EDA Playground:
1.  Ensure the "Testbench + Design" files are selected in the left-hand panel.
2.  Select a simulator (e.g., Aldec Riviera Pro).
3.  Check the "Open EPWave after run" box to view waveforms.
4.  Click the "Run" button.
