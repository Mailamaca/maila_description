# Maila description (Mailamaca)

This package contains the Xacro macro describing the Maila geometry

![maila](_media/rviz.png =250x)

## Usage

```sh
ros2 launch maila_description robot_description_publisher.launch.py
```

The script loads the *maila.urdf.xacro* robot description and start to publish
it on the */robot_description* topic. Addittionally it also subscribe to the
joint states and publish the correct *tf* for them.

Inside the folder is also stored a *dummy_state_publisher.py* that emulates
the hardware by publishing fake sensor data.By running it in combination with
*rviz2* it is possile to see the car moving in the odom frame while the
wheels are spinning.

