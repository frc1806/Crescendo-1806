package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class FieldOrientedDrive extends Command{

    private Swerve mSwerve;
    private DriverControls mDriverControls;
    private Timer mTimer;
    private double mPreciseRotationTarget;

    public FieldOrientedDrive(){
        addRequirements(RobotContainer.S_SWERVE);
        mSwerve = RobotContainer.S_SWERVE;
        mDriverControls = RobotContainer.S_DRIVERCONTROLS;
        mTimer = new Timer();
        mPreciseRotationTarget = mSwerve.getSwerveDrive().getOdometryHeading().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.disableHeadingCorrection();
    }

    @Override
    public void execute() {

        if(!mDriverControls.wantPreciseRotation())
        {
            mSwerve.driveFieldOriented(mSwerve.getTargetSpeedsFromPreScaledInputs(
                mDriverControls.translationX(),
                mDriverControls.translationY(),
                Rotation2d.fromDegrees(mDriverControls.snapRotation())));
            mPreciseRotationTarget = mSwerve.getSwerveDrive().getOdometryHeading().getDegrees();
        }
        else{
            mPreciseRotationTarget += (mDriverControls.getPreciseRotationAxis() * (180.0 * mTimer.get()));
            
            mSwerve.driveFieldOriented(mSwerve.getTargetSpeedsFromPreScaledInputs(
                mDriverControls.translationX(),
                mDriverControls.translationY(),
                Rotation2d.fromDegrees(mPreciseRotationTarget)));
        }
        mTimer.restart();
    }

    @Override
    public void initialize() {
        mPreciseRotationTarget = mSwerve.getSwerveDrive().getOdometryHeading().getDegrees();
        mSwerve.enableHeadingCorrection();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
