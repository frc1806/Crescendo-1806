package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Driver Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kDebugControllerPort = 2;

    public static final double kAcceptableRotationStickError = 1.0/32.0;
    public static final double kRightXboxJoystickDeadzone = 0.1;
    public static final double kLeftXboxJoystickDeadzone = 0.07;
    public static final double kTriggerXboxDeadzone = 0.05;

    // Swerve
    public static final double kMaxModuleSpeed = 5.1;
    public static final double kDriveBaseRadius = 6.8125;
    // Translation PID
    public static final double kSwerveAutoPIDP = 2.5;
    public static final double kSwerveAutoPIDI = 0.0;
    public static final double kSwerveAutoPIDD = 0.001;

    // Angler
    public static final double kAnglerP = 1024.0 / ((32.0/ 360.0) * 4096.0) ; //100% power at 30 degrees away.
    public static final double kAnglerI = 0.0;
    public static final double kAnglerD = 0.0;
    public static final double kAnglerCruiseVelocity = (70.0 / 360.0) * 4096.0;
    public static final double kAnglerMaxAcceleration = (60.0 / 360.0) * 4096.0;
    public static final double kAcceptableAngleError = 6.0;
    public static final double kAnglerGearRatio = 1.0; //to find out //unused?
    public static final double kSpeakerHeight = 1.828; //Meters
    public static final double kAnglerTwistDetectionAngleDifference = 5.0;

    // Vision

    // Pose Estimator Constants
    public static final Transform3d kFrontLeftCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(14-1.25), Units.inchesToMeters(4), Units.inchesToMeters(7.5+1.75)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(20))
    );
    public static final Transform3d kFrontRightCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(14-1.25), Units.inchesToMeters(-4), Units.inchesToMeters(7.5+1.75)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-20))
    );
    public static final Transform3d kBackLeftCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(11), Units.inchesToMeters(7.5+5.5)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(160))
    );
    public static final Transform3d kBackRightCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(11), Units.inchesToMeters(7.5+5.5)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-160))
    );

    // Swerve Drive Pose Estimator Constants
    public static final SwerveModulePosition[] kSwerveModulePositions = {};
    public static final Vector<N3> kStateStds = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Vector<N3> kVisionStds = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    // Launcher
    public static final double kLeftLauncherkP = (1/1000);
    public static final double kLeftLauncherkI = 0.0;
    public static final double kLeftLauncherkD = 0.0;
    public static final double kLeftLauncherkF = (1.15/6000);
    public static final double kRightLauncherkP = (1/1000);
    public static final double kRightLauncherkI = 0.0;
    public static final double kRightLauncherkD = 0.0;
    public static final double kRightLauncherkF = (1.15/5600);
    public static final double kLauncherAcceptableSpeedTolerance = 200; //RPM

    // LED
    public static final int kNumLED = 100; // TODO: placeholder
    public static final double kTwinkleOffAnimationSpeed = 0.75;
    public static final double kLarsonAnimationSpeed = 0.5;
    public static final double kColorFlowAnimationSpeed = 0.5;
    public static final double kStrobeAnimationSpeed = 0.5;
    public static final double kRGBFadeAnimationSpeed = 0.5;
}
