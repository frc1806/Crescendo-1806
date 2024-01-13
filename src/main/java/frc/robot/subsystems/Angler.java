package frc.robot.subsystems;



import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angler extends SubsystemBase {
    
    private CANSparkFlex mAnglerMotor;
    private PIDController mPidController;
    private double mCurrentDesiredAngle;
    private double mCurrentAngle;
    private double mWantedManualPower;

    public Angler() {
        mAnglerMotor = new CANSparkFlex(21, CANSparkFlex.MotorType.kBrushless);
        mPidController = new PIDController(Constants.kAnglerP, Constants.kAnglerI, Constants.kAnglerD);
        mAnglerMotor.setIdleMode(IdleMode.kBrake);
        mCurrentAngle = mAnglerMotor.getEncoder().getPosition();
    }

    public double getAngle() {
        return mCurrentAngle;
    }

    public void setAngle(double angle) {
        mCurrentDesiredAngle = angle;
    }
}
