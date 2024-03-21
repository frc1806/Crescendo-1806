package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Launcher extends SubsystemBase{

    private CANSparkFlex mLauncherLeftMotor;
    private CANSparkFlex mLauncherRightMotor;
    private SparkPIDController mLeftLauncherPID, mRightLauncherPID;
    private CANSparkFlex mIndexLeader;
    private CANSparkFlex mIndexFollower;
    private SparkLimitSwitch mOuterLauncherPhotoEye;
    private SparkLimitSwitch mInnerLauncherPhotoEye;
    private SparkLimitSwitch mIndexerPhotoEye;
    RelativeEncoder mLeftSparkEncoder;
    RelativeEncoder mRightSparkEncoder;

    private double mLauncherTargetSpeed;
    public Launcher(){
        mLauncherLeftMotor = new CANSparkFlex(RobotMap.kLauncherLeaderId, CANSparkFlex.MotorType.kBrushless);
        mLauncherLeftMotor.setInverted(true);
        mOuterLauncherPhotoEye = mLauncherLeftMotor.getForwardLimitSwitch(Type.kNormallyOpen); 
        mOuterLauncherPhotoEye.enableLimitSwitch(false);
        mInnerLauncherPhotoEye = mLauncherLeftMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        mInnerLauncherPhotoEye.enableLimitSwitch(false);
        mLauncherRightMotor = new CANSparkFlex(RobotMap.kLauncherFollowerId, CANSparkFlex.MotorType.kBrushless);
        mLeftLauncherPID = mLauncherLeftMotor.getPIDController();
        mRightLauncherPID = mLauncherRightMotor.getPIDController();
        mLeftLauncherPID.setP(Constants.kLeftLauncherkP);
        mLeftLauncherPID.setI(Constants.kLeftLauncherkI);
        mLeftLauncherPID.setD(Constants.kLeftLauncherkD);
        mLeftLauncherPID.setFF(Constants.kLeftLauncherkF);
        
        mRightLauncherPID.setP(Constants.kRightLauncherkP);
        mRightLauncherPID.setI(Constants.kRightLauncherkI);
        mRightLauncherPID.setD(Constants.kRightLauncherkD);
        mRightLauncherPID.setFF(Constants.kRightLauncherkF);
        

        mIndexLeader = new CANSparkFlex(RobotMap.kIndexerLeaderId, CANSparkFlex.MotorType.kBrushless);
        mIndexerPhotoEye = mIndexLeader.getReverseLimitSwitch(Type.kNormallyOpen);
        mIndexerPhotoEye.enableLimitSwitch(true);

        mIndexFollower = new CANSparkFlex(RobotMap.kIndexerFollowerId, CANSparkFlex.MotorType.kBrushless);
        mIndexFollower.follow(mIndexLeader, true);
        mLeftSparkEncoder = mLauncherLeftMotor.getEncoder();
        mRightSparkEncoder = mLauncherRightMotor.getEncoder();


        setLauncherMode(IdleMode.kBrake);
        setIndexerMode(IdleMode.kBrake);
    }

    public void setLauncher(double speed){
        mLauncherTargetSpeed = speed;
        var mRightLauncherTargetSpeed = speed;

        if(speed>0)
            mRightLauncherTargetSpeed = speed*0.8;
        mLeftLauncherPID.setReference(mLauncherTargetSpeed, ControlType.kVelocity);
        mRightLauncherPID.setReference(mRightLauncherTargetSpeed, ControlType.kVelocity);
    }

    public boolean isLauncherAtSpeed(){
        return mLauncherTargetSpeed != 0 
            && Math.abs(mLeftSparkEncoder.getVelocity() - mLauncherTargetSpeed) < Constants.kLauncherAcceptableSpeedTolerance;
    }

    public void runMotorsForIntake(){
        mLauncherTargetSpeed = -4000.0;
        setLauncher(mLauncherTargetSpeed);
        mIndexLeader.setVoltage(-3.0);
    }

    public void runMotorsForOuttake(){
        mLauncherTargetSpeed = 3000.0;
        setLauncher(mLauncherTargetSpeed);
        mIndexLeader.setVoltage(6.0);
    }

    public void runMotorsForCleaning(){
        mLauncherTargetSpeed = -1000.0;
        setLauncher(mLauncherTargetSpeed);
        mIndexLeader.setVoltage(-4.0);
    }

    public void sendNoteToFlywheel(){
        mIndexLeader.setVoltage(12.0); //SEND IT
    }

    private  void setLauncherMode(IdleMode mode){
        mLauncherLeftMotor.setIdleMode(mode);
        mLauncherRightMotor.setIdleMode(mode);
    }

    private  void setIndexerMode(IdleMode mode){
        mIndexLeader.setIdleMode(mode);
        mIndexFollower.setIdleMode(mode);
    }

    public boolean isNoteAtEndOfLauncher(){
        return mOuterLauncherPhotoEye.isPressed() && !mInnerLauncherPhotoEye.isPressed();
    }

    public boolean isLauncherAndIndexerEmpty(){
        return !mIndexerPhotoEye.isPressed() && !mInnerLauncherPhotoEye.isPressed() && !mOuterLauncherPhotoEye.isPressed();
    }

    public boolean isNoteInIndexer(){
        return mIndexerPhotoEye.isPressed();
    }

    public void stopIndexer(){
        mIndexLeader.set(0.0);
    }

    public void stop(){
        setLauncher(0.0);
        stopIndexer();
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Left Launcher Speed", mLeftSparkEncoder.getVelocity());
        SmartDashboard.putNumber("Right Launcher Speed", mRightSparkEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher Target Speed", mLauncherTargetSpeed);
        SmartDashboard.putBoolean("Launcher Rear Note Sensor", isNoteInIndexer());
    }

    @Override
    public void simulationPeriodic() {
    }



}
