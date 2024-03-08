// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class FieldOrientedRadDrive extends Command {

    private Swerve mSwerve;
    private DriverControls mDriverControls;
    private Timer mTimer;
    private double mPreciseRotationTarget;

    public FieldOrientedRadDrive(Swerve swerve, DriverControls driverControls){
        addRequirements(swerve);
        mSwerve = swerve;
        mDriverControls = driverControls;
        mTimer = new Timer();
        mPreciseRotationTarget = mSwerve.getSwerveDrive().getOdometryHeading().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {

        if(!mDriverControls.wantPreciseRotation())
        {
                mSwerve.driveFieldOriented(mSwerve.getTargetSpeeds(
                    mDriverControls.getRadDriveTranslation().getX(),
                    mDriverControls.getRadDriveTranslation().getY(),
                    Rotation2d.fromDegrees(mDriverControls.snapRotation())));
            mPreciseRotationTarget = mSwerve.getSwerveDrive().getOdometryHeading().getDegrees();
        }
        else{
            mPreciseRotationTarget += (mDriverControls.getPreciseRotationAxis() * (180.0 * mTimer.get()));
            
            mSwerve.driveFieldOriented(mSwerve.getTargetSpeeds(
                mDriverControls.getRadDriveTranslation().getX(),
                mDriverControls.getRadDriveTranslation().getY(),
                Rotation2d.fromDegrees(mPreciseRotationTarget)));
        }
        mTimer.restart();
    }

    @Override
    public void initialize() {
        mPreciseRotationTarget = mSwerve.getSwerveDrive().getOdometryHeading().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
