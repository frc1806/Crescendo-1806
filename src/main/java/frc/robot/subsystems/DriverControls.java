package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.RotateDrive;

public class DriverControls {
    
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
        return (driverController.getRightY() > Constants.kRightStickDeadzone);
    }

    public boolean rotateAbout(){
        // Rotate the robot facing backwards field relative
        return (driverController.getRightY() < -Constants.kRightStickDeadzone);
    }

    public boolean rotateRight(){
        // Rotate the robot facing right field relative
        return (driverController.getRightX() > Constants.kRightStickDeadzone);
    }

    public boolean rotateLeft(){
        // Rotate the robot facing left field relative
        return (driverController.getRightX() < -Constants.kRightStickDeadzone);
    }

    private boolean lockPods(){
        return driverController.getXButton();
    }

    public void registerTriggers(Swerve swerve){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
        new Trigger(this::rotateCenter).onTrue(new RotateDrive(0, swerve, this));
        new Trigger(this::rotateAbout).onTrue(new RotateDrive(180, swerve, this));
        new Trigger(this::rotateLeft).onTrue(new RotateDrive(-90, swerve, this));
        new Trigger(this::rotateRight).onTrue(new RotateDrive(90, swerve, this));
    }

}
