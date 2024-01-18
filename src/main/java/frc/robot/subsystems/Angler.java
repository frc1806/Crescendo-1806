package frc.robot.subsystems;



import com.revrobotics.CANSparkFlex;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.game.Shots;

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
        mCurrentDesiredAngle = Shots.Home.getPivotAngle();
        mCurrentAngle = mAnglerMotor.getEncoder().getPosition();
    }

    public double getAngle() {
        return mCurrentAngle;
    }

    public void setAngle(double angle) {
        mCurrentDesiredAngle = angle; 
    }

    public void setWantedManualPower(double wantedPower){
        mWantedManualPower = wantedPower;
    }

    public boolean atPosition(){
        return Math.abs(getAngle()-mCurrentDesiredAngle) < Constants.kAcceptableAngleError;
    }

    public void setMotor(Double num){
        mAnglerMotor.setVoltage(num * 12);
    }

}
