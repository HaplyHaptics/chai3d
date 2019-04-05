# CHAI3D - The Open Source Haptic Framework

## Introduction

First launched in 2003 at the Robotics and Artificial Intelligence Laboratory at Stanford University, CHAI3D is a powerful cross-platform C++ simulation framework with over 100+ industries and research institutions developing CHAI3D based applications all around the world in segments such as automotive, aerospace, medical, entertainment and industrial robotics.

Designed as a platform agnostic framework for computer haptics, visualization and interactive real-time simulation, CHAI3D is an open source framework that supports a variety of commercially-available three-, six- and seven-degree-of-freedom haptic devices, and makes it simple to support new custom force feedback devices.

CHAI3D's modular capabilities allows for the creation of highly-performing native haptic applications as well as for hybrid development where you can choose which components provide the best haptic and visual user experience.

## Learning Resources

* General information is available at the CHAI3D Homepage. http://www.chai3d.org
* Community discussion takes place on the CHAI3D Forum. http://www.chai3d.org/forum/index
* Commercial support is available from Force Dimension. http://www.forcedimension.com
* Doxygen-generated reference documentation is available online. http://www.chai3d.org/download/doc/html/wrapper-overview.html
* Instructions on how to get started can be found in the documentation folder: [doc/getting-started.html](doc/getting-started.html)

## Modules

Support and examples for extension frameworks can be found in the dedicated module folders:

* [modules/BULLET](modules/BULLET)
* [modules/GEL](modules/GEL)
* [modules/OCULUS](modules/OCULUS) (Visual Studio 2015 only)
* [modules/ODE](modules/ODE)
* [modules/V-REP](modules/V-REP) 

## Haptic Devices and Trackers

CHAI3D supports a selection of commercial haptic devices and trackers. 
Information about their installation and support within CHAI3D can be found in the documentation folder: [doc/html/chapter2-installation.html](doc/html/chapter2-installation.html)

### Haply devices support

This Chai3d distribution features support for Haply devices using [Haply-API-cpp](https://github.com/HaplyHaptics/Haply-API-cpp). If you enable Haply devices support, the Chai3d distribution is then licensed under the terms ofthe [GPLv3 License](LICENSE-Haply-API-cpp.md). 

To enable Haply device support: 
 * Update git submodules in the cloned repository: `git submodule update --init --recursive`
 * Compile with CMake using your favorite integrated development environment or terminal, with CMake option `ENABLE_HAPLY_DEVICES` set to `ON`.

## Contributing

If you have developed a new module or improved any of the capabilities of CHAI3D and would like to share them with the community, please contact our development team at [developers@chai3d.org](mailto:developers@chai3d.org)

## Reporting Bugs

If you have found a bug:
1. Please join the CHAI3D forum and ask about the expected and observed behaviors to determine if it is really a bug
2. If you have identified a problem and managed to document it, you may also submit a report at [support@chai3d.org](support@chai3d.org)

## License

CHAI3D is open source software and is licensed under the Revised BSD License (3-clause). The license allows unlimited redistribution for any purpose as long as its copyright notices and the license's disclaimers of warranty are maintained. The license also contains a clause restricting use of the names of contributors for endorsement of a derived work without specific permission. 

See [copyright.txt](copyright.txt) for details.

## Reference

For scientific publications, please reference CHAI3D:

```
@INPROCEEDINGS{Conti03,
  author    = {Conti, F. and Barbagli, F. and Balaniuk, R. 
               and Halg, M. and Lu, C. and Morris, D. 
               and Sentis, L. and Warren, J. and Khatib, O. 
               and Salisbury, K.},
  title     = {The CHAI libraries},
  booktitle = {Proceedings of Eurohaptics 2003},
  year      = {2003},
  pages     = {496--500},
  address   = {Dublin, Ireland}
}
```

If you publish work using Chai3d with the Haply device (including ports and transpilations to other development languages), we kindly ask you to properly acknowledge our work by citing the publication related to the first showcase of the first demo using Haply devices with Chai3d: 

```
@inproceedings{FrissonFreesoundTrackerHAID2019,
 author = {Frisson, Christian and Gallacher, Colin and Wanderley, Marcelo M.},
 title = {Haptic techniques for browsing sound maps organized by similarity},
 booktitle = {International Workshop on Haptic and Audio Interaction Design},
 series = {HAID},
 year = {2019},
 month = {Mar},
 pages = {1},
 address = {Lille, France},
 url = {https://hal.archives-ouvertes.fr/hal-02050235},
 pdf = {https://hal.archives-ouvertes.fr/hal-02050235/file/demo3.pdf},
 hal_id = {hal-02050235},
 hal_version = {v1},
}
```


## Copyright

(C) 2003-2019 by CHAI3D

(C) 2019 by Haply

All Rights Reserved.

## Acknowledgements

Haply device support in Chai3d has been undertaken as part of [Christian Frisson](http://frisson.re)'s postdoctoral fellowship with [Marcelo M. Wanderley](http://idmil.org) at McGill University, supported by [Mitacs](https://www.mitacs.ca) Elevate grant IT12555 co-funded by [Haply Robotics](http://haply.co).
