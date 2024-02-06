package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Launcher extends SubsystemBase{

    private CANSparkFlex mLauncherLeader;
    private CANSparkFlex mLauncherFollower;
    private SparkPIDController mSparkPIDController;
    private CANSparkFlex mIndexLeader;
    private CANSparkFlex mIndexFollower;
    private SparkLimitSwitch mOuterLauncherPhotoEye;
    private SparkLimitSwitch mInnerLauncherPhotoEye;
    private SparkLimitSwitch mIndexerPhotoEye;

    private double mLauncherTargetSpeed;
    public Launcher(){
        mLauncherLeader = new CANSparkFlex(RobotMap.kLauncherLeaderId, CANSparkFlex.MotorType.kBrushless);
        mOuterLauncherPhotoEye = mLauncherLeader.getForwardLimitSwitch(Type.kNormallyClosed); //TODO: Verify photosensor switch type
        mOuterLauncherPhotoEye.enableLimitSwitch(false);
        mInnerLauncherPhotoEye = mLauncherLeader.getReverseLimitSwitch(Type.kNormallyClosed);
        mInnerLauncherPhotoEye.enableLimitSwitch(false);
        mLauncherFollower = new CANSparkFlex(RobotMap.kLauncherFollowerId, CANSparkFlex.MotorType.kBrushless);
        mLauncherFollower.follow(mLauncherLeader, true);
        mSparkPIDController = mLauncherLeader.getPIDController();
        mSparkPIDController.setP(Constants.kLauncherkP);
        mSparkPIDController.setI(Constants.kLauncherkI);
        mSparkPIDController.setD(Constants.kLauncherkD);
        mSparkPIDController.setFF(Constants.kLauncherkF);

        mIndexLeader = new CANSparkFlex(RobotMap.kIndexerLeaderId, CANSparkFlex.MotorType.kBrushless);
        mIndexerPhotoEye = mIndexLeader.getReverseLimitSwitch(Type.kNormallyClosed);
        mIndexerPhotoEye.enableLimitSwitch(true);

        mIndexFollower = new CANSparkFlex(RobotMap.kIndexerFollowerId, CANSparkFlex.MotorType.kBrushless);
        mIndexFollower.follow(mIndexLeader, true);


        setLauncherMode(IdleMode.kBrake);
        setIndexerMode(IdleMode.kBrake);
    }

    public void setLauncher(double speed){
        mLauncherTargetSpeed = speed;
        mSparkPIDController.setReference(mLauncherTargetSpeed, ControlType.kVelocity);
    }

    public boolean isLauncherAtSpeed(){
        return mLauncherTargetSpeed != 0 
            && Math.abs(mLauncherLeader.getEncoder().getVelocity() - mLauncherTargetSpeed) < Constants.kLauncherAcceptableSpeedTolerance;
    }

    public void runMotorsForIntake(){
        setLauncher(-3000.0);
        mIndexLeader.set(-0.5);
    }

    public void sendNoteToFlywheel(){
        mIndexLeader.set(1.0); //SEND IT
    }

    private  void setLauncherMode(IdleMode mode){
        mLauncherLeader.setIdleMode(mode);
        mLauncherFollower.setIdleMode(mode);
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
    }

    @Override
    public void simulationPeriodic() {
    }



}
