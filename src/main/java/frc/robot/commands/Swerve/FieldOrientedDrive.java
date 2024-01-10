package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class FieldOrientedDrive extends Command{

    private Swerve mSwerve;
    private DriverControls mDriverControls;

    public FieldOrientedDrive(Swerve swerve, DriverControls driverControls){
        addRequirements(swerve);
        mSwerve = swerve;
        mDriverControls = driverControls;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        mSwerve.driveFieldOriented(mSwerve.getTargetSpeeds(mDriverControls.translationX(), mDriverControls.translationY(), new Rotation2d(mDriverControls.rotation())));
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
