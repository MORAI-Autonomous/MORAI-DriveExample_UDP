[![MORAILog](./docs/MORAI_Logo.png)](https://www.morai.ai)
===
# MORAI - Drive example (UDP)

First step to enjoy the `MORAI Sim: Drive` with UDP.

```
./
├── autonomous_driving     # [Autonomous Driving] autonomous driving module
├── network                # UDP network connection
│    ├── receiver            # UDP network - receiver functions
│    ├── sender              # UDP network - sender functions
│    ├── config.json         # UDP network connection config file
│    └── udp_manager.py      # UDP network manager class
└── main.py                # [Entry] example excuter
```

These example contains the below list.
  - Trajectory following lateral control
  - Smart(adaptive) Cruise Control
  - UDP communication

# Requirement

- python >= 3.7

# Installation

Install packages which basically need

```
$ git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_UDP.git
$ cd MORAI-DriveExample_UDP
$ git submodule update --init --recursive
$ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
```

# Usage

Enjoy the example which follow the trajectory with smart cruise control.
```
$ py ./main.py
```

# License
- MORAI Drive Example license info:  [Drive Example License](./docs/License.md)
- MORAI Autonomous Driving license info: [Autonomous Driving License](./autonomous_driving/docs/License.md)
- MGeo Module license info: [MGeo module License](./autonomous_driving/mgeo/lib/mgeo/docs/License.md)

