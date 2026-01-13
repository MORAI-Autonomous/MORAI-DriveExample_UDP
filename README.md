[![MORAILog](./docs/MORAI_Logo.png)](https://www.morai.ai)
===
# MORAI - Closed-loop drive example (UDP)

This example demonstrates autonomous driving capabilities including trajectory following, adaptive cruise control, and UDP-based communication with the MORAI simulator.

## Repo Structure

```
./
├── autonomous_driving     # External submodule: autonomous driving module
├── network                # UDP network connectors
│    ├── receiver            # UDP receiver functions
│    ├── sender              # UDP sender functions
│    ├── config.json         # UDP network configuration file
│    └── udp_manager.py      # UDP network manager class
└── main.py                # Main entry function
```

## Features

- **Trajectory Following**: Lateral control using Pure Pursuit
- **Adaptive Cruise Control**: Smart longitudinal control with object detection
- **UDP Communication**: Real-time data exchange with MORAI Simulator
- **Configurable Parameters**: Easy customization via JSON configuration

## Setup requirements

- Python >= 3.7
- MORAI SIM: Drive

## Setup

1. Clone the repository with submodules:

```bash
$ git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_UDP.git
$ cd MORAI-DriveExample_UDP
$ git submodule update --init --recursive
```

2. Install required Python packages:

```bash
$ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
```

3. Or install packages individually:

```bash
$ pip install -r requirements.txt
$ pip install -r autonomous_driving/requirements.txt
```

### Alternative setup tools

We recommend [uv](https://github.com/astral-sh/uv) a new python environment management tool for easy setup.

```bash
$ uv init
$ uv run main.py 
```

## Use

Examine the network configuration file `network/config.json` and match the values to the configuration values in your MORAI SIM instance.

| Parameter | Description | Default |
|-----------|-------------|---------|
| `network.user_ip` | IP address for user (receiver) | "127.0.0.1" |
| `network.host_ip` | IP address for host (sender) | "127.0.0.1" |
| `network.ego_info_host_port` | Port for receiving ego vehicle information (host) | 908 |
| `network.ego_info_dst_port` | Port for receiving ego vehicle information (destination) | 909 |
| `network.object_info_host_port` | Port for receiving object information (host) | 7605 |
| `network.object_info_dst_port` | Port for receiving object information (destination) | 7505 |
| `network.get_traffic_host_port` | Port for receiving traffic light status (host) | 7603 |
| `network.get_traffic_dst_port` | Port for receiving traffic light status (destination) | 7502 |
| `network.ctrl_cmd_host_port` | Port for sending control commands (host) | 9095 |
| `network.ctrl_cmd_dst_port` | Port for sending control commands (destination) | 9096 |
| `network.set_traffic_host_port` | Port for sending traffic light commands (host) | 7607 |
| `network.set_traffic_dst_port` | Port for sending traffic light commands (destination) | 7503 |

Run the example by running the main entry function.

```bash
$ python ./main.py
```

If successful, the main ego vehicle in your MORAI SIM instance should start to autonomously drive around the scene.

## Resources

- [Website](https://www.morai.ai/)
- **Documentation**:
  - [MORAI SIM Manual (English)](https://morai-sim-drive-user-manual-en-24-r2.scrollhelp.site/morai-sim-drive-user-manual-en-24.r2/Working-version/?l=en)
  - [MORAI SIM Manual (Korean)](https://help-morai-sim.scrollhelp.site/)

## License

- MORAI Drive Example license info:  [Drive Example License](./docs/License.md)
- MORAI Autonomous Driving license info: [Autonomous Driving License](./autonomous_driving/docs/License.md)
- MGeo Module license info: [MGeo module License](./autonomous_driving/mgeo/lib/mgeo/docs/License.md)

