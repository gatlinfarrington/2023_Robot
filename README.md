# 2023_Robot
FRC Team 1165 Robot code, 2023 Charged Up Competition


## TODO

** Once Robot Design Decisions are becoming more finalized **
1. Determine all motors, sensors, pnuematics, etc. that will be on robot:
  Current Working List:
    Drivetrain Motors: Falcon500s
    Drivetrain Encoders: Falcon500 integrated.
2. Determine Controls of Robot
  -Ideally all controls are kept on a single controller.
3. Identify code Subsystems and Corresponding Commands
  i.e Subsystem - Arm, Commands: setGroundLevel, setMidLevel, setHighLevel
4. Split Subsystems and Commands amongst students working on code, students should work on commands that relate to the subsystem they are working on.
  Commands are where the knowledge gained from 2023_Sensor_Project and 2023_Vision_Project will come into play.
5. Autonomous
  Different Autonomous options:
    1. Cross line (left and right starting spots)
    2. Score and cross line (left and right starting spots)
    3. score and balance (middle starting position)
    4. Score, cross line pickup new piece (left and right starting spots)
    5. score, cross line, pickup, drive back (left and right starting spots)
    6. score, cross, pickup, drive back, score again. (left and right starting spots)

## State Machine Example:
https://github.com/Sabercat-Robotics-4146-FRC/Robot_Code-2019/blob/master/src/main/java/frc/robot/TeleopControls.java
