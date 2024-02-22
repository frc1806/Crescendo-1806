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
import frc.robot.game.Shot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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


        updateAnglesFromSensors();
        mCurrentDesiredAngle = Shot.HOME.getPivotAngle();

        isAnglerEnabled = true;
    }

    public void goToPosition(double wantedAngle){
        if(isAnglerEnabled)
        {
        double wantedSensorValue = convertAngleToSensorValue(wantedAngle);
        mAnglerMotorLeft.set(ControlMode.MotionMagic, wantedSensorValue);
        mAnglerMotorRight.set(ControlMode.MotionMagic, wantedSensorValue);
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
        mRightSensorAngle = convertAngleToSensorValue(mAnglerMotorRight.getSelectedSensorPosition());
        mTwistAngle = mLeftSensorAngle - mRightSensorAngle;
        mCurrentAngle = (mLeftSensorAngle + mRightSensorAngle) / 2.0;
    }

    private double convertAngleToSensorValue(double angle){
        return (angle / 360.0) * 4096.0;
    }

    private double convertSensorValueToAngle(double sensorValue){
        return (sensorValue / 4096.0) * 360.0;
    }

    private boolean isTwistDetected(){
        return Math.abs(mTwistAngle) > Constants.kAnglerTwistDetectionAngleDifference;
    }

    private boolean isEitherSensorOutsideAcceptableRange(){
        return (mLeftSensorAngle > 360.0 || mLeftSensorAngle < 0.0 || mRightSensorAngle > 360.0 || mRightSensorAngle < 0.0);
    }

    private boolean hasSomethingGoneHorriblyWrong(){
        return isTwistDetected() || isEitherSensorOutsideAcceptableRange();
    }


    public static boolean isBlueTeam(){
        if(DriverStation.getAlliance().isPresent()){
            return DriverStation.getAlliance().get() == Alliance.Blue;
        }
        return false;
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

        SmartDashboard.putNumber("Angler Wanted Manual Power", mWantedManualPower);
        //Read encoders
        updateAnglesFromSensors();

        if(hasSomethingGoneHorriblyWrong())
        {
            isAnglerEnabled = false; //TODO: Notify drivers somehow. LEDs? Dashboard? Rumble?
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
