# Smart Warehouse CSMA/CA Simulation (ns-3.39)

This simulation script models a $30 \times 30$ meter smart warehouse environment using **Wi-Fi 802.11n Ad-hoc** mode (without RTS/CTS). The system consists of **1 stationary Gateway** located at the center and **$N$ Mobile Robots** moving randomly according to the *Random Waypoint* mobility model while transmitting continuous UDP CBR traffic to the Gateway.

---

## 1. Prerequisites

* **Operating System:** Linux (Ubuntu 20.04 / 22.04 LTS or equivalent)
* **Software:** Installed **ns-3.39** along with its build dependencies (`build-essential`, `python3`, `cmake`, etc.).

---

## 2. Installation & Running Instructions

### Step 1: Copy the source code to the `scratch` directory
Place your C++ source code file into the `scratch` folder located inside your ns-3 root directory (e.g., `ns-3.39/scratch/`):

1. Save your C++ code as `smart-warehouse-csma-ca.cc`
2. Move the file into the `scratch` directory:
   ```bash
   cp smart-warehouse-csma-ca.cc <ns3_path>/scratch/
