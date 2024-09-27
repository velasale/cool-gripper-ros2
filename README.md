# cool-gripper-ros2




## ARDUINO
If the Arduino IDE is not able to upload programs to boards, it may be because the user does not have permission to access boards.
Fix it by:

```console
sudo usermod -a -G dialout <user>
```