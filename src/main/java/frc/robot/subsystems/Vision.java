package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import java.util.function.DoubleSupplier;

public class Vision extends SubsystemBase {
    
    private PhotonCamera mFrontLeftCam, mFrontRightCam, mBackLeftCam, mBackRightCam;
    private PhotonPoseEstimator mFrontLeftEstimator, mFrontRightEstimator, mBackLeftEstimator, mBackRightEstimator;
    private FlippableBlueAlliancePose mBlueSpeakerPose;
    private FlippableBlueAlliancePose mBlueAmpPose;
    private BlueAllianceFieldPoseCollection mBlueAllianceTrapPoses;
    private DoubleSupplier mVAngle;
    private DoubleSupplier mVSpeed;
    private Shot visionShot;
    
    private AprilTagFieldLayout mFieldLayout;

    private static final Swerve SWERVE = RobotContainer.S_SWERVE;

    public Vision(){
        mFrontLeftCam = new PhotonCamera("FrontLeft");
        mFrontRightCam = new PhotonCamera("FrontRight");
        mBackLeftCam = new PhotonCamera("BackLeft");
        mBackRightCam = new PhotonCamera("BackRight");

        mFrontLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontLeftCam, Constants.kFrontLeftCamToCenter);
        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.kFrontRightCamToCenter);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.kBackLeftCamToCenter);
        mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackRightCam, Constants.kBackRightCamToCenter);

        mFrontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mBlueSpeakerPose = new FlippableBlueAlliancePose(new Translation2d(0.02, 5.61), Rotation2d.fromDegrees(180));
        mBlueAmpPose = new FlippableBlueAlliancePose(new Translation2d(1.84, 7.64), Rotation2d.fromDegrees(90.0));

        ArrayList<FlippableBlueAlliancePose> blueTrapPoses= new ArrayList<>();
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.36, 4.92), Rotation2d.fromDegrees(-60.0)));
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.41, 3.33), Rotation2d.fromDegrees(60.0)));
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(5.78, 4.1), Rotation2d.fromDegrees(180.0)));
        
        mBlueAllianceTrapPoses = new BlueAllianceFieldPoseCollection(blueTrapPoses);

        mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
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

        if(mFrontLeftEstimator.update().isPresent()){
            RobotContainer.S_SWERVE.addVisionMeasurement(mFrontLeftEstimator.update().get().estimatedPose.toPose2d(), mFrontLeftEstimator.update().get().timestampSeconds);
        }
        if(mFrontRightEstimator.update().isPresent()){
            RobotContainer.S_SWERVE.addVisionMeasurement(mFrontRightEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds);
        }
        if(mBackLeftEstimator.update().isPresent()){
            RobotContainer.S_SWERVE.addVisionMeasurement(mBackLeftEstimator.update().get().estimatedPose.toPose2d(), mBackLeftEstimator.update().get().timestampSeconds);
        }
        if(mBackRightEstimator.update().isPresent()){
            RobotContainer.S_SWERVE.addVisionMeasurement(mBackRightEstimator.update().get().estimatedPose.toPose2d(), mBackRightEstimator.update().get().timestampSeconds);
        }
    }

    public void resetRobotOdometryFromVision(){
        boolean hasBeenReset = false;
        if(mFrontLeftEstimator.update().isPresent()){
            hasBeenReset= true;
            RobotContainer.S_SWERVE.resetOdometry(mFrontLeftEstimator.update().get().estimatedPose.toPose2d());
        }
        if(mFrontRightEstimator.update().isPresent()){
            if(hasBeenReset){
                RobotContainer.S_SWERVE.addVisionMeasurement(mFrontRightEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds);
            }
            else{
                hasBeenReset= true;
                RobotContainer.S_SWERVE.resetOdometry(mFrontRightEstimator.update().get().estimatedPose.toPose2d());
            }
        }
        if(mBackLeftEstimator.update().isPresent()){
                        if(hasBeenReset){
                RobotContainer.S_SWERVE.addVisionMeasurement(mBackLeftEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds);
            }
            else{
                hasBeenReset= true;
                RobotContainer.S_SWERVE.resetOdometry(mBackLeftEstimator.update().get().estimatedPose.toPose2d());
            }
        }
        if(mBackRightEstimator.update().isPresent()){
                        if(hasBeenReset){
                RobotContainer.S_SWERVE.addVisionMeasurement(mBackRightEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds);
            }
            else{
                hasBeenReset= true;
                RobotContainer.S_SWERVE.resetOdometry(mBackRightEstimator.update().get().estimatedPose.toPose2d());
            }
        }
    }



    @Override
    public void simulationPeriodic() {
    }

}
