# Troubleshooting

### Invalid frame ID "map" passed to canTransform
```bash
[rviz2-1] Warning: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
[rviz2-1] at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp
``` 
- Everytime you launch the simulation, you're gonna see this warning being printed out aggressively and as a result, you might fail to see the racecar `RobotModel`s. Just give it some time to receive the transforms from buffer. If the console doesn't stop outputting this message after up to a minute, try restarting.
### I can't see any map!
- Make sure you've configured the map_path parameter of `sim.yaml` correctly. It should look like `/path/to/map`. Notice how there is no extension after the map file. That is handled with the `map_img_ext` parameter. If your map file format is a .pgm instead of .png, set `map_img_ext` parameter accordingly.

### I can see the map but there are no racecars. There's only a white rectangle!
- Take a look at the [racecar](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1) repository's `README.md` if you haven't already.
- Try to launch the racecar nodes first, then the simulation. After launching the simulation, wait up to a minute. 
- If you've changed the racecar_namespace parameter, there might be something broken because of that. I recommend changing it back to its original state.

### I can see both the map and the racecars, but the racecars don't move
In your control algorithm, make sure you're publishing and subscribing to correct topics. If you didn't change any parameters in .yaml files (except map path and map image extension parameters), the default topics that are published and subscribed by simulation will be like: 
- `/racecar1/scan` ...(and other topics under racecar1 namespace).
- `/racecar2/odom` ...(and other topics under racecar2 namespace).
- `/racecar3/drive` where `racecar3` is the `racecar_namespace` parameter and `drive` is the `drive_topic` parameter.

Notice that the topics above are the default values of these parameters. If you had set the 

`racecar_namespace` to be `convertible`

`drive_topic` to be `ride`,

`odom_topic` to be `xy`

 and `scan_topic` to be `scanner` in the `sim.yaml` file under the `config` directory, the topics that are being published by the simulation would be:
- `/convertible1/scanner` ...(and other topics under convertible1 namespace).
- `/convertible2/xy` ...(and other topics under convertible2 namespace).
- `/convertible3/ride` ...(and other topics under convertible3 namespace).

It is recommended to leave the parameters under `sim.yaml` with their default values (except map parameters). But if you insist on changing them, make sure you only use alphabetical characters (e.g. `sdv`) And double check the topics you subscribe and publish to in your racecar workspace. 

- An important thing to remind is while publishing a drive message, your `AckermannDriveStamped` message has to have a `header.frame_id` of `{racecar_namespace}/base_link`. Otherwise the simulator won't be able to tell which racecar to drive. For `racecar1` it would be `racecar1/base_link`

### My changes aren't reflecting to the simulation!
- Make sure you rebuild the container or if you're using Ubuntu 20.04 native installation, rebuild the workspace with `colcon build`.


### Rviz is not showing up!
- If you're running the simulation via docker, don't forget to run the xhost command mentioned in the [installation](#with-an-nvidia-gpu) section and run the container with the command mentioned there