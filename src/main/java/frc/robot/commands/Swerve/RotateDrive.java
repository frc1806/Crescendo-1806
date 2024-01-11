package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class RotateDrive extends Command{

    private double mAngle;
    private Swerve mSwerve;
    private DriverControls mDriverControls;

    public RotateDrive(double angle, Swerve swerve, DriverControls driverControls){
        addRequirements(swerve);
        mAngle = angle;
        mSwerve = swerve;
        mDriverControls = driverControls;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        mSwerve.driveFieldOriented(mSwerve.getTargetSpeeds(mDriverControls.translationX(), mDriverControls.translationY(), new Rotation2d(mAngle)));
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
