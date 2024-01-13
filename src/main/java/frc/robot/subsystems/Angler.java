package frc.robot.subsystems;



import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angler extends SubsystemBase {
    
    private CANSparkFlex mAnglerMotor;
    private PIDController mPidController;
    private double mCurrentAngle;
    private double mWantedManualPower;

    public Angler() {
        mAnglerMotor = new CANSparkFlex(21, CANSparkFlex.MotorType.kBrushless);
        mPidController = new PIDController(0.1, 0.0, 0.0);
    }
}
