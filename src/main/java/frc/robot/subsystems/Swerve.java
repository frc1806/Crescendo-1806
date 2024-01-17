package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

    private double maximumSpeed;
    private SwerveDrive swerveDrive;
    private File swerveJsonDirectory;

    public Swerve(){
        maximumSpeed = Units.feetToMeters(14.5);
        swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/testBench");
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (Exception exception){
            throw new RuntimeException("Runtime error when creating a new swerve drive:\n" + exception);
        }

        swerveDrive.setHeadingCorrection(false);

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotOrientedVelocity,
            this::setChassisSpeed,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.kSwerveAutoPIDP, Constants.kSwerveAutoPIDI, Constants.kSwerveAutoPIDD),
                new PIDConstants(
                    swerveDrive.swerveController.config.headingPIDF.p,
                    swerveDrive.swerveController.config.headingPIDF.i,
                    swerveDrive.swerveController.config.headingPIDF.d),
                Constants.kMaxModuleSpeed,
                Units.feetToMeters(Constants.kDriveBaseRadius),
                new ReplanningConfig()
            ),
            this::shouldPathFlip,
            this
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void driveRobotOriented(Translation2d translation, double rotation){
        swerveDrive.drive(translation, rotation, false, false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity){
        swerveDrive.driveFieldOriented(velocity);
    }

    public SwerveDriveKinematics getKinematics(){
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose){
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeed(ChassisSpeeds velocity){
        swerveDrive.setChassisSpeeds(velocity);
    }

    public boolean shouldPathFlip(){
        //Path will not flip if the current alliance is blue alliance
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void postTrajectory(Trajectory trajectory){
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGryo(){
        swerveDrive.zeroGyro();
    }

    public void setBrakeMode(boolean brake){
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY){
      xInput = Math.pow(xInput, 3);
      yInput = Math.pow(yInput, 3);
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(), maximumSpeed);  
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle){
      xInput = Math.pow(xInput, 3);
      yInput = Math.pow(yInput, 3);
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), maximumSpeed);
    }
  
    public ChassisSpeeds getTargetSpeedsFromPreScaledInputs(double xInput, double yInput, Rotation2d angle){
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), maximumSpeed);
    }

    public ChassisSpeeds getFieldOrientedVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotOrientedVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock(){
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch(){
        return swerveDrive.getPitch();
    }

    //Test function for testing vision
    public void addFakeVisionReading(){
        swerveDrive.addVisionMeasurement(new Pose2d(3,3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }

}
