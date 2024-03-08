package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.game.Shot;
import frc.robot.util.rumbleutil.RumbleCommand;
import frc.robot.util.rumbleutil.SquareWave;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angler extends SubsystemBase {
    
    private TalonSRX mAnglerMotorLeft, mAnglerMotorRight;
    private double mCurrentDesiredAngle;
    private double mCurrentAngle;
    private double mLeftSensorAngle;
    private double mRightSensorAngle;
    private double mTwistAngle;
    private double mWantedManualPower;

    private boolean isAnglerEnabled;

    public Angler() {

        //LEFT MOTOR
        mAnglerMotorLeft = new TalonSRX(RobotMap.AnglerLeftMotorId);
        mAnglerMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        mAnglerMotorLeft.configFeedbackNotContinuous(false, 50);
        mAnglerMotorLeft.setInverted(InvertType.None);
        mAnglerMotorLeft.setSensorPhase(false);
        mAnglerMotorLeft.config_kP(0, Constants.kAnglerP);
        mAnglerMotorLeft.config_kI(0, Constants.kAnglerI);
        mAnglerMotorLeft.config_kD(0, Constants.kAnglerD);
        mAnglerMotorLeft.configMotionCruiseVelocity(Constants.kAnglerCruiseVelocity); //degrees per tenth of a second, scaled to encoder units.
        mAnglerMotorLeft.configMotionAcceleration(Constants.kAnglerMaxAcceleration); // degrees per tenth of a second squared scaled to encoder units.
        mAnglerMotorLeft.configMotionSCurveStrength(1);
        mAnglerMotorLeft.set(ControlMode.PercentOutput, 0.0);
        mAnglerMotorLeft.setNeutralMode(NeutralMode.Brake);
        
    

        //RIGHT MOTOR
        mAnglerMotorRight = new TalonSRX(RobotMap.AnglerRightMotorId);
        /*
        mAnglerMotorRight.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        mAnglerMotorRight.configFeedbackNotContinuous(false, 50);
        mAnglerMotorRight.setInverted(InvertType.InvertMotorOutput);
        mAnglerMotorRight.setSensorPhase(false); //Invert sensor relative to motor, not sure if needed yet
        mAnglerMotorRight.config_kP(0, Constants.kAnglerP);
        mAnglerMotorRight.config_kI(0, Constants.kAnglerI);
        mAnglerMotorRight.config_kD(0, Constants.kAnglerD);
        mAnglerMotorRight.configMotionCruiseVelocity(Constants.kAnglerCruiseVelocity); //degrees per tenth of a second, scaled to encoder units.
        mAnglerMotorRight.configMotionAcceleration(Constants.kAnglerMaxAcceleration); // degrees per tenth of a second squared scaled to encoder units.
        mAnglerMotorRight.configMotionSCurveStrength(1);
        */
        //FOLLOWER STYLE
        mAnglerMotorRight.follow(mAnglerMotorLeft, FollowerType.PercentOutput);
        mAnglerMotorRight.setInverted(InvertType.InvertMotorOutput);
        //END FOLLOWER STYLE
        mAnglerMotorRight.set(ControlMode.PercentOutput, 0.0);
        mAnglerMotorRight.setNeutralMode(NeutralMode.Brake);



        updateAnglesFromSensors();

        mAnglerMotorRight.setSelectedSensorPosition(-mAnglerMotorLeft.getSelectedSensorPosition());
        updateAnglesFromSensors();
        mCurrentDesiredAngle = Shot.HOME.getPivotAngle();

        isAnglerEnabled = true;
    }

    public void goToPosition(double wantedAngle){
        if(wantedAngle < 250.0){
            System.out.println("Tried to set a launcher angle of:" + wantedAngle);
            wantedAngle = 250.0;
        }
        if(wantedAngle > 400.0){
            System.out.println("Tried to set a launcher angle of:" + wantedAngle);
            wantedAngle = 400.0;
        }
        if(isAnglerEnabled)
        {
        double wantedSensorValue = convertAngleToSensorValue(wantedAngle);
        mAnglerMotorLeft.set(ControlMode.MotionMagic, wantedSensorValue);
        //mAnglerMotorRight.set(ControlMode.MotionMagic, wantedSensorValue);
        }
        else{
            mAnglerMotorLeft.set(ControlMode.PercentOutput, 0.0);
            //mAnglerMotorRight.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public double getAngle() {
        return mCurrentAngle;
    }

    public void setAngle(double angle) {
        mCurrentDesiredAngle = angle; 
        goToPosition(angle);
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

    private void updateAnglesFromSensors(){
        mLeftSensorAngle = convertSensorValueToAngle(mAnglerMotorLeft.getSelectedSensorPosition());
        mRightSensorAngle = convertSensorValueToAngle(mAnglerMotorRight.getSelectedSensorPosition());
        mTwistAngle = mLeftSensorAngle - mRightSensorAngle;
        mCurrentAngle = (mLeftSensorAngle); //+ mRightSensorAngle) / 2.0;
    }

    private double convertAngleToSensorValue(double angle){
        return (angle / 360.0) * 4096.0;
    }

    private double convertSensorValueToAngle(double sensorValue){
        return (sensorValue / 4096.0) * 360.0;
    }

    private boolean isTwistDetected(){
        return false;
        //return Math.abs(mTwistAngle) > Constants.kAnglerTwistDetectionAngleDifference;
    }

    private boolean isEitherSensorOutsideAcceptableRange(){
        return (mLeftSensorAngle > 450.0 || mLeftSensorAngle < 200.0); //|| mRightSensorAngle > 450.0 || mRightSensorAngle < 200.0);
    }

    private boolean hasSomethingGoneHorriblyWrong(){
        return isTwistDetected() || isEitherSensorOutsideAcceptableRange();
    }

    public void enableAngler(){
        isAnglerEnabled = true;
    }

    public void disableAngler(){
        isAnglerEnabled = false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Angler Angle", mCurrentAngle);
        SmartDashboard.putNumber("Angler Left Sensor Angle", mLeftSensorAngle);
        SmartDashboard.putNumber("Angler Right Sensor Angle", mRightSensorAngle);
        SmartDashboard.putNumber("Angler Twist Angle", mTwistAngle);
        
        SmartDashboard.putNumber("Angler Left Sensor Value", mAnglerMotorLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("Angler Right Sensor Value", mAnglerMotorRight.getSelectedSensorPosition());
        
        SmartDashboard.putNumber("Angler Left Motor Output", mAnglerMotorLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Angler Right Motor Output", mAnglerMotorRight.getMotorOutputPercent());

        SmartDashboard.putBoolean("Angler Enabled", isAnglerEnabled);

        SmartDashboard.putNumber("Angler Wanted Angle", mCurrentDesiredAngle);
        SmartDashboard.putNumber("Angler Wanted Sensor Angle:", convertAngleToSensorValue(mCurrentDesiredAngle));

        SmartDashboard.putNumber("Angler Wanted Manual Power", mWantedManualPower);
        //Read encoders
        updateAnglesFromSensors();

        if(hasSomethingGoneHorriblyWrong())
        {
            if(isAnglerEnabled){
                //Notify Drivers
                //6 long pulses.
                RobotContainer.S_DRIVERCONTROLS.addRumbleCommandToDriverQueue(new RumbleCommand(new SquareWave(1.0, 0.75, 0.7), RumbleType.kRightRumble, 6.0));
                RobotContainer.S_DRIVERCONTROLS.addRumbleCommandToOperatorQueue(new RumbleCommand(new SquareWave(1.0, 0.75, 0.7), RumbleType.kRightRumble, 6.0));
                //TODO: LEDs.
            }
            isAnglerEnabled = false;
        }

        //Run motor with manual contorl if wanted
        if((RobotContainer.S_DRIVERCONTROLS.d_wantAnglerManual() || RobotContainer.S_DRIVERCONTROLS.o_wantManualAnglerRotate())){
                setMotor(mWantedManualPower);
        }
        else
        {
            if(!isAnglerEnabled)
            {
                setMotor(0.0);
            }
        }
    }
}
