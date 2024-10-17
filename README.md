
After building the package, type:
```
ros2 launch interview_homework frames.launch.py 
```
to launch the frame publisher and open rviz

The rotating fram will move with a default linear velocity along the z axis and will rotate about the z axis

You can select these velocity passing the parameters:
```
ros2 launch interview_homework frames.launch.py ang_vel:=desiredAng_vel lin_vel:=desiredLinVel
```

