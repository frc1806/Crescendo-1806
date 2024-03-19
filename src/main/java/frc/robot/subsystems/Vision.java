package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.util.PolarCoordinate;
import frc.robot.util.fieldmirroring.BlueAllianceFieldPoseCollection;
import frc.robot.util.fieldmirroring.FlippableBlueAlliancePose;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;

public class Vision extends SubsystemBase {
    
    private PhotonCamera mFrontLeftCam, mFrontRightCam, mBackLeftCam, mBackRightCam;
    private PhotonPoseEstimator mFrontLeftEstimator, mFrontRightEstimator, mBackLeftEstimator, mBackRightEstimator;
    private FlippableBlueAlliancePose mBlueSpeakerPose, mBlueSubwooferAmp, mBlueSubwooferCenter, mBlueSubwooferAntiAmp, mBlueDriveThruFeeder, mBluePreDriveThruFeeder;
    private FlippableBlueAlliancePose mBlueAmpPose;
    private BlueAllianceFieldPoseCollection mBlueAllianceTrapPoses;
    private DoubleSupplier mVAngle;
    private DoubleSupplier mVSpeed;
    private Shot visionShot;
    private static Translation3d ORIGINTRANSLATION = new Translation3d();
    
    private AprilTagFieldLayout mFieldLayout;

    public Vision(){
        mFrontLeftCam = new PhotonCamera("LeftFront");
        mFrontRightCam = new PhotonCamera("RightFront");
        mBackLeftCam = new PhotonCamera("LeftRear");
        mBackRightCam = new PhotonCamera("RightRear");

        mFrontLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontLeftCam, Constants.kFrontLeftCamToCenter);
        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.kFrontRightCamToCenter);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.kBackLeftCamToCenter);
        mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackRightCam, Constants.kBackRightCamToCenter);

        mFrontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mBlueSpeakerPose = new FlippableBlueAlliancePose(new Translation2d(0.2, 5.55), Rotation2d.fromDegrees(180));
        mBlueSubwooferCenter = new FlippableBlueAlliancePose(new Pose2d(1.36, 5.52, Rotation2d.fromDegrees(180)));
        mBlueSubwooferAntiAmp = new FlippableBlueAlliancePose(new Pose2d(0.76, 4.41, Rotation2d.fromDegrees(120)));
        mBlueSubwooferAmp= new FlippableBlueAlliancePose(new Pose2d(0.76, 6.66, Rotation2d.fromDegrees(-120)));
        mBluePreDriveThruFeeder = new FlippableBlueAlliancePose(new Pose2d(15.94, 2.27, Rotation2d.fromDegrees(-150.0)));
        mBlueDriveThruFeeder = new FlippableBlueAlliancePose(new Pose2d(14.85, 0.74, Rotation2d.fromDegrees(-150.0)));
        mBlueAmpPose = new FlippableBlueAlliancePose(new Translation2d(1.84, 7.64), Rotation2d.fromDegrees(90.0));

        ArrayList<FlippableBlueAlliancePose> blueTrapPoses= new ArrayList<>();
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.36, 4.92), Rotation2d.fromDegrees(-60.0)));
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.41, 3.33), Rotation2d.fromDegrees(60.0)));
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(5.78, 4.1), Rotation2d.fromDegrees(180.0)));
        
        mBlueAllianceTrapPoses = new BlueAllianceFieldPoseCollection(blueTrapPoses);

        mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        mFrontLeftEstimator.setFieldTags(mFieldLayout);
        mFrontRightEstimator.setFieldTags(mFieldLayout);
        mBackLeftEstimator.setFieldTags(mFieldLayout);
        mBackRightEstimator.setFieldTags(mFieldLayout);
    }

    public Rotation2d getYawToSpeaker(){
        Translation2d difference = getYAxisMovementCompensatedSpeakerPose().minus(RobotContainer.S_SWERVE.getPose().getTranslation());
        return Rotation2d.fromRadians(PolarCoordinate.toPolarCoordinate(() -> difference.getX(), () -> difference.getY())[1]);
    }


    public Translation2d getYAxisMovementCompensatedSpeakerPose(){
        return getSpeakerTranslation().plus(new Translation2d(0.0, -(RobotContainer.S_SWERVE.getFieldOrientedVelocity().vyMetersPerSecond * .3)));
    }

    public Translation2d getSpeakerTranslation(){
        return mBlueSpeakerPose.getTranslation(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getAmpGoalPose2d(){
        return mBlueAmpPose.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getNearestTrapPose(){
        return mBlueAllianceTrapPoses.getNearestPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance(), RobotContainer.S_SWERVE.getPose());
    }

    public Double getDistanceToSpeaker(){
        return RobotContainer.S_SWERVE.getPose().getTranslation().getDistance(getSpeakerTranslation());
    }

    public Pose2d getAmpSideSubwoofer(){
        return mBlueSubwooferAmp.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getCenterSubwoofer(){
        return mBlueSubwooferCenter.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getAntiAmpSubwoofer(){
        return mBlueSubwooferAntiAmp.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getPreDriveThruFeeder(){
        return mBluePreDriveThruFeeder.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getDriveThruFeeder(){
        return mBlueDriveThruFeeder.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public DoubleSupplier CalculateShotAngle() {
        visionShot = VisionShotLibrary.getShotForDistance(getDistanceToSpeaker());
        mVAngle = visionShot::getPivotAngle;
        return mVAngle;
    }

    public DoubleSupplier CalculateShotSpeed() {
        visionShot = VisionShotLibrary.getShotForDistance(getDistanceToSpeaker());
        mVSpeed = visionShot::getLauncherSpeed;
        return mVSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);

        try{
            if(mFrontLeftEstimator.update().isPresent() && updateIsValid(mFrontLeftEstimator.update().get())){
                EstimatedRobotPose update = mFrontLeftEstimator.update().orElseThrow();
                if(update != null){
                    RobotContainer.S_SWERVE.addVisionMeasurement(update.estimatedPose.toPose2d(), update.timestampSeconds);
                }
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Front left estimator had no update to get");
        }

        try{
            if(mFrontRightEstimator.update().isPresent() && updateIsValid(mFrontRightEstimator.update().get())){
                EstimatedRobotPose update = mFrontRightEstimator.update().orElseThrow();
                if(update != null){
                    RobotContainer.S_SWERVE.addVisionMeasurement(update.estimatedPose.toPose2d(), update.timestampSeconds);
                }
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Front right estimator had no update to get");
        }
        
         try{
            if(mBackLeftEstimator.update().isPresent() && updateIsValid(mBackLeftEstimator.update().get())){
                EstimatedRobotPose update = mBackLeftEstimator.update().orElseThrow();
                if(update != null){
                    RobotContainer.S_SWERVE.addVisionMeasurement(update.estimatedPose.toPose2d(), update.timestampSeconds);
                }
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Back left estimator had no update to get");
        }
         try{
            if(mBackRightEstimator.update().isPresent() && updateIsValid(mBackRightEstimator.update().get())){
                EstimatedRobotPose update = mBackRightEstimator.update().orElseThrow();
                if(update != null){
                    RobotContainer.S_SWERVE.addVisionMeasurement(update.estimatedPose.toPose2d(), update.timestampSeconds);
                }
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Back right estimator had no update to get");
        }
    }

    public void resetRobotOdometryFromVision(){
        boolean hasBeenReset = false;
        try{
            if(mFrontLeftEstimator.update().isPresent()  && updateIsValid(mFrontLeftEstimator.update().get())){
                hasBeenReset= true;

                RobotContainer.S_SWERVE.resetOdometry(mFrontLeftEstimator.update().get().estimatedPose.toPose2d());
            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
         try{
            if(mFrontRightEstimator.update().isPresent() && updateIsValid(mFrontLeftEstimator.update().get())){
                hasBeenReset= true;
                RobotContainer.S_SWERVE.resetOdometry(mFrontRightEstimator.update().get().estimatedPose.toPose2d());
            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
        try{
            if(mBackLeftEstimator.update().isPresent() && updateIsValid(mBackLeftEstimator.update().get())){
                hasBeenReset= true;
                RobotContainer.S_SWERVE.resetOdometry(mBackLeftEstimator.update().get().estimatedPose.toPose2d());
            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
         try{
            if(mBackRightEstimator.update().isPresent() &&updateIsValid(mBackRightEstimator.update().get())){
                hasBeenReset= true;
                RobotContainer.S_SWERVE.resetOdometry(mBackRightEstimator.update().get().estimatedPose.toPose2d());
            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
    }

    private boolean updateIsValid(EstimatedRobotPose estimatedRobotPose){
        return updateHasMultipleTargets(estimatedRobotPose) && updateHasAllNearTargets(estimatedRobotPose);
    }

    private boolean updateHasMultipleTargets(EstimatedRobotPose estimatedRobotPose){
        return estimatedRobotPose.targetsUsed.size() > 1.0;
    }

    private boolean updateHasAllNearTargets(EstimatedRobotPose estimatedRobotPose){
        for(PhotonTrackedTarget target: estimatedRobotPose.targetsUsed){
            double distanceToTag = target.getBestCameraToTarget().getTranslation().getDistance(ORIGINTRANSLATION); 
            if(distanceToTag > 6.0){
                return false;
            }
        }
        return true;
        
    }


    @Override
    public void simulationPeriodic() {
    }

}
