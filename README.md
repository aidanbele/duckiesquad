# duckiesquad


## duckiebot-mother

Duckiebot-mother acts as the leader, and wanders around

```bash
cd duckiebot-ros-test
dts devel build -f -H reggiequackson.local && dts devel run -H reggiequackson.local
```

## duckiebot-daughter

Duckiebot-daughter follows the mother around. It would be cool if when the daughter loses the mother it will attempt to find her

```bash
cd duckiebot-ros-test
dts devel build -f -H sirquacksalot.local && dts devel run -H sirquacksalot.local
```

### manual control on mac

Should be able to control the robot using this command: (no UI)

```bash
dts duckiebot keyboard_control hostname --cli
```

### duckiebot-ros-test

This is a super basic ROS node that can run on a robot.

Basically, all it does is there is one node that publishes a message once a second, and a second node that subscribes to that message. This is basically the same thing we will need to do to communicate between different parts of our code if we need multiple nodes, and more importantly, communicate with the robot itself, both for getting data in on the camera and for giving it commands.

For local compilation do:

```bash
cd duckiebot-ros-test
dts devel build -f
dts devel run
```

In order to get this to run on an actual duckie bot, we need to use the `-H` flag

```bash
cd duckiebot-ros-test
dts devel build -f -H MY_ROBOT.local
dts devel run -H MY_ROBOT.local
```
