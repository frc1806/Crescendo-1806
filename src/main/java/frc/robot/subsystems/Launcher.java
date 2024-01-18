package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Launcher extends SubsystemBase{

    private CANSparkFlex mLauncherMaster;
    private CANSparkFlex mLauncherSlave;
    private PIDController mPIDController;

    public Launcher(){
        mLauncherMaster = new CANSparkFlex(RobotMap.kLauncherMasterPort, CANSparkFlex.MotorType.kBrushless);
        mLauncherSlave = new CANSparkFlex(RobotMap.kLauncherSlavePort, CANSparkFlex.MotorType.kBrushless);
        mPIDController = new PIDController(Constants.kLauncherkP, Constants.kLauncherkI, Constants.kLauncherkD);
        
        mLauncherSlave.follow(mLauncherMaster);

        setLauncherMode(IdleMode.kBrake);
    }

    public void setLauncher(double speed){

        if(speed != 0.0){
            setLauncherMode(IdleMode.kCoast);
        }
        else {
            setLauncherMode(IdleMode.kBrake);
        }

        mLauncherMaster.set(speed);
        mLauncherSlave.set(speed);
    }

    public void setLauncherMode(IdleMode mode){
        mLauncherMaster.setIdleMode(mode);
        mLauncherSlave.setIdleMode(mode);
    }

    public void stop(){
        setLauncher(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }

}
