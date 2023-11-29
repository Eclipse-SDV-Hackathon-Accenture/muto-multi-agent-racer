# Muto Multi Agent Race Challenge

## About

[F1TENTH Autonomous Racing](https://f1tenth.org/) is a semi-regular competition organized by an international community of researchers, engineers, and autonomous systems enthusiasts. The 11th of these races happened as a part of the [ICRA 2023](https://icra2023-race.f1tenth.org/). You can review some the [ICRA Resources](https://icra2023-race.f1tenth.org/race_ressources.html) to familiarize yourself with the race.  In these races teams build a 1:10 scaled autonomous physical race car according to a given specification and write software for it to fulfill the objectives for the competition: **Don’t crash and minimize laptime**.  

This challenge does not require you to build a race car!  We will provide a simple multiagent simulator for you. But you need to write the race software (or modify the winner from an earlier challenge).

<img src="./assets/overview.png" />

## Simulator 

Autonomous Driving needs heavy development in simulation to provide a good evaluation for the developed algorithms before we bring them on the car. We provide different simulation environments that can help you in your development. For the virtual competitation we will use modified version of the [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) which provides an asynchronous, 2D simulator built in Python. This simulation runs faster than real-time execution (30x realtime), provides a realistic vehicle simulation and collision, runs multiple vehicle instances and publishes laser scan and odometry data.

## Your Goal 
Well you are not limited by anything!!!  For example, you can try to **improve the lap time and do not crash**. You can also use SDV technologies such as [Eclipse Muto], [Eclipse Zenoh] and remotely adjust the drive parameters with a mobile Race App. As a hack challenge for Eclipse SDV, it's a good use case to try out the involved technology and experience some of the challenges when dealing with connected vehicles. You will also get to know a few more open source projects along the way. 
In this hack challenge, you can

- **Build software for faster race car**. A better driving style leads to safer roads, less energy consumption, smaller carbon footprint. For some, it's even an incentive to safe money on insurance costs.
- **Build apps and services**. You can build mobile and cloud apps and services to share race telemetry and control race cars.
- **Explore the Eclipse SDV ecosystem** of open source projects, such as programming models, middleware and protocol implementations.
- **Combine exciting open source technology** from robotics, internet of things, cloud and automotive

The following diagram summarizes some of the Eclipse SDV technologies that you can use to build your racer:

<img src="./assets/racer.png" />

The results of this challenge will be integrated into the ROS Racers SDV blueprint to orchestrate and manage the F1Tenth stacks that power the miniature vehicles racing around a track.

Eclipse Muto orchestrates (install/start/stop/configure..) ros software on racers and data collection from vehicles, providing remote control, race Apps and Services using SDV stacks. It also provides a default agent (based on Eclipse Ditto) and a UI.

## Need to know

No formal requirements are present for this challenge, yet certain degree of knowledge in following could be extremely useful. Your hack team should have the following skills:

- Some **development skills** in a programming language - Python, GoLang, Rust, C++ or even Java are good candidates.
- Rudimentary **Linux, git, shell skills** are always a plus. Some knowledge in network protocols such as MQTT is good. You don't need to be an expert in automotive specific tech such as CAN-Bus, Some/IP or AutoSAR.
- Compiling, building and cross-compilation basics are needed as well. Your app may run on your x86-based development laptop, but does it also run on the ARM-based target device?
- A bit of **Docker, containerd, k8s** experience would be good as well, for building your app as a container and deploy it to a container runtime on the device.
- If you're in for some more advanced stuff, run your application on the Xen Hypervisor together with a ROS based system.

As we want to focus on the technology exploration and integration, consider the following steps as exemplary, you're free to chose your own set of technology of course.

## Getting Started

- Links:
  + [Twin Server](http://ditto:ditto@sandbox.composiv.ai). You could add, remove and observe stacks, view API documentation etc.
  + [Muto Dashboard](https://ditto:ditto@dashboard.composiv.ai). You could manage stacks without having to go through API documentation and manually doing it yourself

- [Step 1 - Installation](docs/step1-installation.md)

- [Step 2 - Getting started with the F1Tenth Gym](docs/step2-getting-started-with-f1tenth-gym.md)

- [Step 3 - Explore sample racer](docs/step3-make-racecar-interact-with-f1tenth-gym.md)

- [Step 4 - Exploring utilities to visualize the data](docs/step4-rviz.md)

- [Step 5 - Building ROS apps and services with Zenoh](docs/step5-build-zenoh.md)

Happy hacking!



## Hack-Coaches

- Naci Dai
- Ibrahim Sel
- Deniz Memis
