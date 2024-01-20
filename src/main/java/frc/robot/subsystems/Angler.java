package frc.robot.subsystems;



import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.game.Shots;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angler extends SubsystemBase {
    
    private CANSparkFlex mAnglerMotor;
    private PIDController mPidController;
    private double mCurrentDesiredAngle;
    private double mCurrentAngle;
    private double mWantedManualPower;
    private DutyCycleEncoder mAnglerEncoder;

    public Angler() {
        mAnglerMotor = new CANSparkFlex(RobotMap.kAnglerMotorPort, CANSparkFlex.MotorType.kBrushless);
        mAnglerMotor.getEncoder().setPositionConversionFactor(1/Constants.kAnglerGearRatio * 360);
        mPidController = new PIDController(Constants.kAnglerP, Constants.kAnglerI, Constants.kAnglerD);
        mAnglerMotor.setIdleMode(IdleMode.kBrake);
        mCurrentDesiredAngle = Shots.Home.getPivotAngle();
        mCurrentAngle = mAnglerMotor.getEncoder().getPosition();
        mAnglerEncoder = new DutyCycleEncoder(RobotMap.kAnglerEncoderPort);
        mAnglerEncoder.setDutyCycleRange(1.0/1025.0,  1024.0/1025.0);
        mAnglerEncoder.setDistancePerRotation(360.0);
    }

    public void resetMotorEncoderToAbsoluteEncoder(){
        double proposedNewPos = mAnglerEncoder.getDistance();
        if(proposedNewPos < 0)
        {
            proposedNewPos +=360;
        }
        mAnglerMotor.getEncoder().setPosition(proposedNewPos);
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
        public CANSparkFlex getPivotMotor(){
        return mAnglerMotor;
    }
    
    public DutyCycleEncoder getEncoder(){
        return mAnglerEncoder;
    }

        @Override
    public void periodic(){
        mCurrentAngle = mAnglerMotor.getEncoder().getPosition();
        
        if(!(RobotContainer.S_DRIVERCONTROLS.d_wantAnglerManual() || RobotContainer.S_DRIVERCONTROLS.o_wantManualAnglerRotate())){
            if (atPosition()){
                //true stuff here
                setMotor(0.0);
            }
            else{
                //false stuff here
                setMotor(mPidController.calculate(getAngle(), mCurrentDesiredAngle) * 12);
            }
            if(RobotState.isDisabled())
            {
                resetMotorEncoderToAbsoluteEncoder();
                mCurrentDesiredAngle = mCurrentAngle;
            }
        }
        else{
            setMotor(mWantedManualPower * 3);
        }
    }


}
