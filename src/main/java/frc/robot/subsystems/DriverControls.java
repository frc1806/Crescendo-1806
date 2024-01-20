package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.PreciseRotateDrive;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.SnapRotateDrive;
import frc.robot.commands.Swerve.VisionRotateDrive;

public class DriverControls{
    
    /*
     * My unusual naming conventions documentation:
     * methodName means the method is tied to the driver controller
     * oMethodName means the method is tied to the operator controller
     * dMethodName means the method is tied to the debug controller
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
        return -driverController.getLeftX();
    }
    
    public double translationX(){
        return -driverController.getLeftY();
    }

    public boolean rotateCenter(){
        // Rotate the robot facing forwards field relative
        return (driverController.getRightY() > 0) && !wantPreciseRotation();
    }

    public boolean rotateAbout(){
        // Rotate the robot facing backwards field relative
        return (driverController.getRightY() < 0)  && !wantPreciseRotation();
    }

    public boolean rotateRight(){
        // Rotate the robot facing right field relative
        return (driverController.getRightX() > 0) && !wantPreciseRotation();
    }

    public boolean rotateLeft(){
        // Rotate the robot facing left field relative
        return (driverController.getRightX() < 0) && !wantPreciseRotation();
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

    public void registerTriggers(Swerve swerve, Intake intake){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
        new Trigger(this::rotateCenter).onTrue(new SnapRotateDrive(180, swerve, this));
        new Trigger(this::rotateAbout).onTrue(new SnapRotateDrive(0, swerve, this));
        new Trigger(this::rotateLeft).onTrue(new SnapRotateDrive(-90, swerve, this));
        new Trigger(this::rotateRight).onTrue(new SnapRotateDrive(90, swerve, this));
        new Trigger(this::resetGyro).onTrue(new ResetGyro(swerve));
        new Trigger(this::intake).whileTrue(new SetIntake(intake, this, 1));
        new Trigger(this::wantPreciseRotation).onTrue(new PreciseRotateDrive(swerve, this));
        new Trigger(this::wantVisionAlign).whileTrue(new VisionRotateDrive(swerve, this));
    }



}
