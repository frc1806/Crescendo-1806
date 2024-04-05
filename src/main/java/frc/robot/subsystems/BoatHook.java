package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class BoatHook extends SubsystemBase{
    
    private TalonSRX mBoatHookMotor;

    private double mBoatHookWantedSpeed;

    public BoatHook(){
        mBoatHookMotor = new TalonSRX(RobotMap.kBoatHookMotor);
    }


    public Command ExtendBoatHook(){

      return this.run(() -> {
            mBoatHookWantedSpeed = 0.75;
            mBoatHookMotor.set(ControlMode.PercentOutput, mBoatHookWantedSpeed);
        });


    }

    public Command StopBoatHook(){

      return this.run(() -> {
            mBoatHookWantedSpeed = 0.0;
            mBoatHookMotor.set(ControlMode.PercentOutput, mBoatHookWantedSpeed);
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
