package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class BoatHook extends SubsystemBase{
    
    private CANSparkMax mBoatHookMotor;

    public BoatHook(){
        mBoatHookMotor = new CANSparkMax(RobotMap.kBoatHookMotorId, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }
    
}
