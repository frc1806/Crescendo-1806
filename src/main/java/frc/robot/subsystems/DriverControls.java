package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.VisionShot;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.VisionRotateDrive;
import frc.robot.util.PolarCoordinate;

public class DriverControls{
    
    /*
     * My unusual naming conventions documentation:
     * methodName means the method is tied to the driver controller
     * o_MethodName means the method is tied to the operator controller
     * d_MethodName means the method is tied to the debug controller
    */

    private XboxController driverController;
    private XboxController operatorController;
    private XboxController debugController;

    public DriverControls(){
        driverController = new XboxController(Constants.kDriverControllerPort);
        operatorController = new XboxController(Constants.kOperatorControllerPort);
        debugController = new XboxController(Constants.kDebugControllerPort);
    }

    public XboxController getDriverController(){
        return driverController;
    }

    public XboxController getOperatorController(){
        return operatorController;
    }

    public XboxController getDebugController(){
        return debugController;
    }

    // DRIVER CONTROLS

    public double translationY(){
        return -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kLeftXboxJoystickDeadzone);
    }
    
    public double translationX(){
        return -MathUtil.applyDeadband(driverController.getLeftY(), Constants.kLeftXboxJoystickDeadzone);
    }

    public int snapRotation(){
        double[] rightJoyPolarCoordinate = PolarCoordinate.toPolarCoordinate(driverController::getRightX, driverController::getRightY);
        double r = MathUtil.applyDeadband(rightJoyPolarCoordinate[0], Constants.kRightXboxJoystickDeadzone);
        double theta = Units.radiansToDegrees(rightJoyPolarCoordinate[1]);
    
        if(r < Constants.kRightXboxJoystickDeadzone){
            RobotContainer.S_SWERVE.getSwerveDrive().getOdometryHeading().getDegrees();
        }

        if(theta < 22.5 && theta >= 337.5){
            return 270;
        }
        else if(theta < 67.5 && theta >= 22.5){
            return 315;
        }
        else if(theta < 112.5 && theta >= 67.5){
            return 0;
        }
        else if(theta < 157.5 && theta >= 112.5){
            return 45;
        }
        else if(theta < 202.5 && theta >= 157.5){
            return 90;
        }
        else if(theta < 247.5 && theta >= 202.5){
            return 135;
        }
        else if(theta < 292.5 && theta >= 247.5){
            return 180;
        }
        else{
            return 225;
        }
    }

    public boolean wantSnapRotation(){
        return MathUtil.applyDeadband(driverController.getRightX(), Constants.kRightXboxJoystickDeadzone)>0
        || MathUtil.applyDeadband(driverController.getRightY(), Constants.kRightXboxJoystickDeadzone)>0;
    }

    public boolean wantPreciseRotation(){
        return driverController.getRightTriggerAxis() > 0;
    }

    public boolean intake(){
        return driverController.getRightBumper();
    }

    public boolean lockPods(){
        return driverController.getXButton();
    }

    public boolean resetGyro(){
        return driverController.getYButton();
    }

    public boolean wantVisionAlign(){
        return driverController.getAButton();
    }

    // OPERATOR CONTROLS
    public boolean o_wantVisionShot(){
        return operatorController.getXButton();
    }

    public boolean o_wantManualAnglerRotate(){
        return operatorController.getRightTriggerAxis() > 0;
    }

    // Debug Controls
    public double d_pivotAnglerManual() {
        return debugController.getRightY();
    }

    public boolean d_wantAnglerManual() {
        return d_pivotAnglerManual() != 0;
    }

    public void registerTriggers(Swerve swerve, Reel intake, Angler angler, Vision vision, Launcher launcher){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
        new Trigger(this::resetGyro).onTrue(new ResetGyro(swerve));
        new Trigger(this::intake).whileTrue(new SetIntake(intake, this, 1));
        new Trigger(this::wantVisionAlign).whileTrue(new VisionRotateDrive(swerve, this));

        //Operator
        new Trigger(this::o_wantVisionShot).whileTrue(new VisionShot(angler, launcher, this));


    }



}
