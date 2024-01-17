package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class VisionRotateDrive extends Command{

    private Swerve mSwerve;
    private DriverControls mDriverControls;

    public VisionRotateDrive(Swerve swerve, DriverControls driverControls){
        mSwerve = swerve;
        mDriverControls = driverControls;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        mSwerve.visionAlign();
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return !mDriverControls.wantVisionAlign();
    }
    
}
