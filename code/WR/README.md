# WaveRunner code and configuration

This directory holds code and configuration for the WR. The source code authored by the project team resides under `src/` and `inc/`,
`lib/` hold third-party libraries used in the program.

## Compiling

In order to build the code running on the WR, run the following commands:

```
mkdir build && cd build
cmake ..
make -j4
```

Now, an executable named `ftl` has been created in the `build` directory. This can be run manually by issuing `./ftl`.
The system is setup to run `ftl` on boot (see below), this instance of the program need to be stopped before starting the program
manually.

## Administration

A few configuration files and scripts are used to setup the Xavier for the purposes of the project, these can be found under
`etc/`. Information about these files and where they should be installed in the system is 

- `ftl.service`
  
  *Installation location:* `/etc/systemd/system/`
  
  *Description:* This is a [systemd](https://www.freedesktop.org/wiki/Software/systemd/) service file, responsible for autostarting
  the program upon starting the system. Basic usage is `sudo systemctl status ftl` to query the status of the system, 
  `sudo systemctl stop ftl` to stop the system and `sudo systemctl start ftl` to start it.
  For further instructions on using systemd, see e.g. [ArchWiki](https://wiki.archlinux.org/index.php/Systemd).
  
- `templog.service`

  *Installation location:* `/etc/systemd/system`
  
  *Description:* Systemd service file for the temperature logger.
  
- `???.sh`

  *Installation location:* `/etc/`
  
  *Description:* Configure the relevant pins for CAN functionality ([documentation](https://github.com/hmxf/can_xavier)).
