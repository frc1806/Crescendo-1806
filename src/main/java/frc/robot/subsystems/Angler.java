package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.game.Shots;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angler extends SubsystemBase {
    
    private TalonSRX mAnglerMotor;
    private PIDController mPidController;
    private double mCurrentDesiredAngle;
    private double mCurrentAngle;
    private double mWantedManualPower;
    private DutyCycleEncoder mAnglerEncoder;

    public Angler() {
        mAnglerMotor = new TalonSRX(RobotMap.kAnglerMotorPort);
        mAnglerEncoder = new DutyCycleEncoder(RobotMap.kAnglerEncoderPort);
        mAnglerMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        mAnglerMotor.setSelectedSensorPosition(0);
        mAnglerMotor.getSensorCollection().getPulseWidthRiseToFallUs();
        mAnglerMotor.set(ControlMode.Position, 10 * 4096);
        mPidController = new PIDController(Constants.kAnglerP, Constants.kAnglerI, Constants.kAnglerD);
        mAnglerMotor.setNeutralMode(NeutralMode.Brake);
        mCurrentDesiredAngle = Shots.Home.getPivotAngle();
        mCurrentAngle = mAnglerMotor.getSelectedSensorPosition();
        mAnglerEncoder.setDutyCycleRange(1.0/1025.0,  1024.0/1025.0);
        mAnglerEncoder.setDistancePerRotation(180.0);
    }

    public void resetMotorEncoderToAbsoluteEncoder(){
        double proposedNewPos = mAnglerEncoder.getDistance();
        if(proposedNewPos < 0)
        {
            proposedNewPos +=360;
        }
        mAnglerMotor.setSelectedSensorPosition(proposedNewPos);
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
        mAnglerMotor.set(ControlMode.PercentOutput, num);
    }
        public TalonSRX getPivotMotor(){
        return mAnglerMotor;
    }
    
    public DutyCycleEncoder getEncoder(){
        return mAnglerEncoder;
    }

        @Override
    public void periodic(){
        mCurrentAngle = mAnglerMotor.getSelectedSensorPosition();
        
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

        //seline bradley is a homonculus!!!!!
    }

}
