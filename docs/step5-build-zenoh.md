# Building ROS Apps and Services with Eclipse Zenoh

The zenoh/DDS bridge is leveraging CycloneDDS to discover the DDS readers and writers declared by the ROS2 application. For each discovered DDS entity the bridge creates a mirror DDS-entity — in other terms it creates a reader when discovering a writer and vice-versa. Additionally, the bridge maps the DDS topics read and written by the discovered DDS entities on zenoh resources and performs the proper declarations.

- Using [Eclipse Zenoh](https://github.com/eclipse-zenoh/zenoh), you can try to send `AckermannDrive` messages to the sim using a mobile app.
- Some ROS2 demo implementations of [Eclipse Zenoh](https://github.com/eclipse-zenoh/zenoh-demos/tree/master/ROS2) can be found here.