## About this package
This package implements both Passive UHF RFID tags and readers as sensors under gazebo-11, without needing to fully recompile gazebo. The implemented model is based on the friis equation, including the noise sources described in the log-distance path loss model. 

If you want to use it with ROS, you will need `gazebo_rfid_node` as well to publish tag detections in a topic, but the aim is to merge that package with this one.
The package uses the same information structure we created for the package `rfid_node`, efectively mocking it under gazebo. The other dependency is `gazebo_custom_sensor_preloader`, which allows to dynamically load sensors into gazebo. 

If you want to quickly test it, use the launcher `test.launch` in the package `gazebo_rfid_node`. It will create an empty world with just a tag and an antenna in reading range. You can play adding obstacles and/or moving the tag to see how the received power and phase changes. Detections should be published in ros topic `/lastTag`.

## See also
This package was inspired by Martin Peka's custom gazebo sensor preloader and rotating lidar sensor implementation. If you want to implement another custom sensor, you should check his work: [gazebo sensor preloader](https://github.com/peci1/gazebo_custom_sensor_preloader) and the example, [rotating lidar](https://github.com/peci1/gazebo_rotating_lidar)

This particular sensor was initially a crude adaptation of gazebo `WirelessTransmitter` and `WirelessReceiver` sensors. Some of their asumptions were taken, so it's also worth checking [here](https://github.com/osrf/gazebo/tree/gazebo11/gazebo/sensors).


## Citing ( a.k.a. sharing is loving)

If you like this code and you are considering using it, please consider citing our work ["Next-Best-Sense: a multi-criteria robotic exploration strategy for RFID tags discovery"](https://ieeexplore.ieee.org/abstract/document/9113679) 

Or, if you are just looking for custom gazebo sensors, you should definitively cite [Martin Peka](https://github.com/peci1)!

## TO-DO
Non comprehensive list of open issues:

- gazebo rfid msg: 
I tried the custom msg tutorial here: http://gazebosim.org/tutorials?tut=custom_messages but couldn't make it work. The message is defined in folder `msgs`, but its compilation is comented in the main CMakeLists file and the code uses a string message instead. This is a nice feature to add, but not quite relevant really. 

- ros plugin:
Currently we publish data from gazebo sensor into a ROS topic by creating a node that subscribes to gazebo topic and publishes into ROS *outside* gazebo (see `gazebo_rfid_node`). The elegant solution should be a gazebo plugin but I couldn't make it work either, there is a runtime error. . The code is there, but commented from the main CMakeLists file and the urdf models.

- hardcoded elements:
I introduced an antenna model, taken from the distributor radiattion pattern. The original model is 3D, but I've only considered the azimuth plane. Tag is considered to be isotropic with a fixed gain no matter the angle. A nice improvement would be to allow parametric antenna models including both altitude and azimuth for both tag and antenna. 

Noise model parameters are also hardcoded. Another nice improvement would be adding parametric noise variance in both phase and power. 

Other propagation models could be considered, now that we have extracted the sensor model from the whole gazebo core.
