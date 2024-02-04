package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.game.Shots;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angler extends SubsystemBase {
    
    private TalonSRX mAnglerMotorLeft, mAnglerMotorRight;
    private double mCurrentDesiredAngle;
    private double mCurrentAngle;
    private double mWantedManualPower;
    private double mDistancex;
    private double mDistancey;
    private double mTotalDistance;
    private Translation2d mBlueSpeakerPose;

    public Angler() {

        //LEFT MOTOR
        mAnglerMotorLeft = new TalonSRX(RobotMap.AnglerLeftMotorId);
        mAnglerMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        mAnglerMotorLeft.configFeedbackNotContinuous(false, 50);
        mAnglerMotorLeft.config_kP(0, Constants.kAnglerP);
        mAnglerMotorLeft.config_kI(0, Constants.kAnglerI);
        mAnglerMotorLeft.config_kD(0, Constants.kAnglerD);
        mAnglerMotorLeft.configMotionCruiseVelocity((40.0 / 360.0) * 4096.0); //degrees per tenth of a second, scaled to encoder units.
        mAnglerMotorLeft.configMotionAcceleration((40.0 / 360.0) * 4096.0); // degrees per tenth of a second squared scaled to encoder units.
        mAnglerMotorLeft.set(ControlMode.PercentOutput, 0.0);
        mAnglerMotorLeft.setNeutralMode(NeutralMode.Brake);
        
    

        //RIGHT MOTOR
        mAnglerMotorRight = new TalonSRX(RobotMap.AnglerRightMotorId);
        mAnglerMotorRight.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        mAnglerMotorRight.configFeedbackNotContinuous(false, 50);
        mAnglerMotorRight.setInverted(InvertType.InvertMotorOutput);
        mAnglerMotorRight.setSensorPhase(true); //Invert sensor relative to motor, not sure if needed yet
        mAnglerMotorRight.config_kP(0, Constants.kAnglerP);
        mAnglerMotorRight.config_kI(0, Constants.kAnglerI);
        mAnglerMotorRight.config_kD(0, Constants.kAnglerD);
        mAnglerMotorRight.configMotionCruiseVelocity((40.0 / 360.0) * 4096.0); //degrees per tenth of a second, scaled to encoder units.
        mAnglerMotorRight.configMotionAcceleration((40.0 / 360.0) * 4096.0); // degrees per tenth of a second squared scaled to encoder units.
        mAnglerMotorRight.set(ControlMode.PercentOutput, 0.0);
        mAnglerMotorRight.setNeutralMode(NeutralMode.Brake);


        mCurrentAngle = getAngleFromSensors();
        mCurrentDesiredAngle = Shots.HOME.getPivotAngle();

        mBlueSpeakerPose = new Translation2d(0.06, 5.55);
    }

    public void goToPosition(double angle){
        mCurrentDesiredAngle = angle;
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
        mAnglerMotorLeft.set(ControlMode.PercentOutput, num);
    }

    public void setMotorToAngle(Double angle){
        mAnglerMotorLeft.set(TalonSRXControlMode.MotionMagic, convertAngleToSensorValue(angle));
    }
        public TalonSRX getPivotMotor(){
        return mAnglerMotorLeft;
    }

    private double getAngleFromSensors(){
        return (mAnglerMotorLeft.getSelectedSensorPosition() + mAnglerMotorRight.getSelectedSensorPosition()) / ((4096.0/360.0) * 2.0);
    }

    private double convertAngleToSensorValue(double angle){
        return (angle / 360.0) * 4096.0;
    }


    public void toSpeakerPose(){
        mDistancex =  RobotContainer.S_VISION.getEstimatedPose().getTranslation().getX() - mBlueSpeakerPose.getX();
        mDistancey =  RobotContainer.S_VISION.getEstimatedPose().getTranslation().getY() - mBlueSpeakerPose.getY();
        mTotalDistance = Math.sqrt(Math.pow(mDistancex, 2) + Math.pow(mDistancey, 2));
        mCurrentDesiredAngle = Math.toDegrees(Math.atan(Constants.kSpeakerHeight/mTotalDistance));
        goToPosition(mCurrentDesiredAngle);
    }



    @Override
    public void periodic(){
        mCurrentAngle = getAngleFromSensors();
        
        if(!(RobotContainer.S_DRIVERCONTROLS.d_wantAnglerManual() || RobotContainer.S_DRIVERCONTROLS.o_wantManualAnglerRotate())){
            if (atPosition()){
                //true stuff here
                setMotor(0.0);
            }
            else{
                setAngle(mCurrentDesiredAngle);;
            }
            if(RobotState.isDisabled())
            {
                mCurrentDesiredAngle = mCurrentAngle;
            }
        }
        else{
            setMotor(mWantedManualPower * 3);
        }

    }

}
