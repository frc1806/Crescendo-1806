package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class BoatHook extends SubsystemBase{
    
    private CANSparkMax mBoatHookMotorLeader;
    private CANSparkMax mBoatHookMotorFollower;

    private double mBoatHookWantedSpeed;

    public BoatHook(){
        mBoatHookMotorLeader = new CANSparkMax(RobotMap.kBoatHookMotorLeaderId, CANSparkMax.MotorType.kBrushless);
        mBoatHookMotorFollower = new CANSparkMax(RobotMap.kBoatHookMotorFollowerId, CANSparkMax.MotorType.kBrushless);
        mBoatHookMotorFollower.follow(mBoatHookMotorLeader);

    
    }

    public Command extendBoatHook(){

      return this.run(() -> {
            mBoatHookWantedSpeed = 0.5;
            mBoatHookMotorLeader.set(mBoatHookWantedSpeed);
        });


    }

    public Command stopBoatHook(){

        return this.run(() -> {
            mBoatHookWantedSpeed = 0.0;
            mBoatHookMotorLeader.set(mBoatHookWantedSpeed);
        });

    }

    public Command retractBoatHook(){

        return this.run(() -> {
            mBoatHookWantedSpeed = -0.5;
            mBoatHookMotorLeader.set(mBoatHookWantedSpeed);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }
    
}
