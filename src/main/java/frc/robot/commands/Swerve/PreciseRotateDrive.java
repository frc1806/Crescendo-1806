package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class PreciseRotateDrive extends Command {

    private Swerve mSwerve;
    private DriverControls mDriverControls;
    private double angle;

    public PreciseRotateDrive(Swerve swerve, DriverControls driverControls){
        mSwerve = swerve;
        mDriverControls = driverControls;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        angle += (mDriverControls.getDriverController().getRightX() * 4);
        mSwerve.driveFieldOriented(mSwerve.getTargetSpeeds(mDriverControls.translationX(), mDriverControls.translationY(), Rotation2d.fromDegrees(angle)));
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return !mDriverControls.wantPreciseRotation();
    }
    
}
