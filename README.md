# Team S.W.A.T. 1806 Robot Code
Here is the robot code that S.W.A.T. is using for the 2024 season. Code will be adapted as our meetings go on

## Subsystems
`src.main.java.frc.robot.subsystems`

- `Angler.java`: Subsystem that sets our shooter to our wanted angles
- `BoatHook.java`: Subsystem used for climbing
- `DriverControls.java`: Subsystem that has all the controls for our two Xbox Controllers. This is also has a method that registers all of our triggers.
- `Launcher.java`: Subsystem that launches notes into the goals
- `LED.java`: Subsystem that controls LEDs
- `Reel.java`: Subsystem that intakes the notes into our shooter
- `Swerve.java`: Our swerve drive subsystem. Powered by YAGSL
- `Vision.java`: Subsystem that uses our vision cameras to detect April Tags on the field. Powered by PhotonLib