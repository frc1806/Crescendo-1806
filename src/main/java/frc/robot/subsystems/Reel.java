package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Reel extends SubsystemBase{
    
    private CANSparkMax mIntakeMotor;
    
    public Reel(){
        mIntakeMotor = new CANSparkMax(RobotMap.kIntakeMotorId, MotorType.kBrushless);
    }

    public Command setIntake(double speed){
        return this.run(() -> mIntakeMotor.set(speed));
    }

    public void stop(){
        setIntake(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }

}
