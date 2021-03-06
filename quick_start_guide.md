## Quick Start Guide:

1. Plug in Robot Controller 230V/50Hz
2. Switch on Robot Controller
3. Set IP of PC's Ethernet Adapter to 192.168.0.2
4. Connect PC with required software (see software dependency list) to Ethernet Port X3 of Robot Controller
7. Connect Modbus/USB Adapter from Gripper to PC
8. Start Matlab
9. Start the script "gripper_init" (Might require setting the correct COM Port of the gripper e.g. "COM3"), the gripper will now open and close to calibrate
10. Switch the robot from manual mode into automatic mode
11. Confirm in the teach panel
12. Confirm that the workspace is free of obstacles and it is safe to operate the robot
13. Push "Activate Motors" Button on the robot controller
14. Place taskboard and make it ready for testrun
15. Open the production menu on the robots teach panel
16. Select "set programm pointer to main" in the robots teach panel
17. Hit play on the robots teach panel, the robot will move into camera position if not already in it
18. Hit play on the matlab script "image_processing"
19. The programm will start as soon as the matlab script sends the detected taskboard pose to the robot
20. Hit stop in the matlab script AND the robots teach panel after each run (for safety reasons)
21. Begin from step 14. to repeat.
