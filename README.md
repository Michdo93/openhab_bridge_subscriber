# openhab_bridge_subscriber
Subscribes states from openhab using a bridge between openHAB and ROS with HABApp

## Installation

Go to your src folder of your catkin_ws and clone the repository:

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/openhab_bridge_subscriber.git
cd ~/catkin_ws
catkin_make
```

## Usage

You can run each subscriber like following:

```
rosrun openhab_bridge_subscriber ColorSubscriber.py
rosrun openhab_bridge_subscriber ContactSubscriber.py
rosrun openhab_bridge_subscriber DateTimeSubscriber.py
rosrun openhab_bridge_subscriber DimmerSubscriber.py
rosrun openhab_bridge_subscriber GroupSubscriber.py
rosrun openhab_bridge_subscriber ImageSubscriber.py
rosrun openhab_bridge_subscriber LocationSubscriber.py
rosrun openhab_bridge_subscriber NumberSubscriber.py
rosrun openhab_bridge_subscriber RollershutterSubscriber.py
rosrun openhab_bridge_subscriber StringSubscriber.py
rosrun openhab_bridge_subscriber SwitchSubscriber.py
```
