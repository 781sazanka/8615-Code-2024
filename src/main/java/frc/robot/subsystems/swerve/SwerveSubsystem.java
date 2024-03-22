package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.Vision;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  /** Swerve drive object. */
  public final SwerveDrive swerveDrive;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public double maximumSpeed = Units.feetToMeters(14.5);
  SwerveDrivePoseEstimator poseEstimator;

  Vision vision = new Vision();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    setupPathPlanner();
    swerveDrive.setHeadingCorrection(false);
    poseEstimator = new SwerveDrivePoseEstimator(
        swerveDrive.kinematics,
        swerveDrive.getOdometryHeading(),
        swerveDrive.getModulePositions(),
        new Pose2d(0, 0, new Rotation2d(0)));
  }

  public SwerveSubsystem(
      SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);

  }

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your
            // Constants class
            AutonConstants.TRANSLATION_PID,
            // Translation PID constants
            AutonConstants.ANGLE_PID,
            // Rotation PID constants
            4.5,
            // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
          // Make the robot move
          driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
              headingX.getAsDouble(),
              headingY.getAsDouble(),
              swerveDrive.getOdometryHeading().getRadians(),
              swerveDrive.getMaximumVelocity()));

        });
  }

  public Command simDriveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble() * Math.PI,
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumVelocity()));
        });
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        maximumSpeed);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        maximumSpeed);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either
   * open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and
   * robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in
   *                      meters per second. In robot-relative mode, positive x is
   *                      torwards the bow (front) and
   *                      positive y is torwards port (left). In field-relative
   *                      mode, positive x is away from the
   *                      alliance wall (field North) and positive y is torwards
   *                      the left wall when looking through
   *                      the driver station glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by
   *                      field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {

    // poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d("limelight"),
    // edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

    // poseEstimator.update(
    // swerveDrive.getOdometryHeading(), swerveDrive.getModulePositions());

    SmartDashboard.putNumber("[Swerve] rotation in degrees",
        swerveDrive.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("[Swerve] robot rotation (*vision)",
        LimelightHelpers.getBotPose2d("limelight").getRotation().getDegrees());
    SmartDashboard.putNumber("[Swerve] target rotation (*vision)", Units.degreesToRadians(
        LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getRotation().getAngle()));
    SmartDashboard.putBoolean("[Swerve] target valid", LimelightHelpers.getTV("limelight"));

    poseEstimator.update(getHeading(), swerveDrive.getModulePositions());
    if (vision.getTV()) {
      poseEstimator.addVisionMeasurement(vision.getEstimatedRoboPose(), vision.getTimestamp(),
          vision.getStandardDeviations());
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  public Command aimAtTarget(LimelightHelpers limelight) {
    double rotation = LimelightHelpers.getBotPose2d("limelight").getRotation().getDegrees();
    return run(() -> {
      drive(getTargetSpeeds(0,
          0,
          Rotation2d.fromDegrees(rotation)));
    });
  }
}