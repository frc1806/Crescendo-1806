# Team S.W.A.T. 1806 Robot Code
Here is the robot code that S.W.A.T. is using for the 2024 season. Code will be adapted as our meetings go on

Currently the project is using the YAGSL `swervelib` folder from the YAGSL-Example repository in the 2024 branch as there is no official release for the library yet

## Subsystems
`src.main.java.frc.robot.subsystems`

`Swerve.java`: Our swerve drive subsystem. Powered by YAGSL
`DriverControls.java`: A class that has all the controls for our two Xbox Controllers. This is also has a method that registers all of our triggers.