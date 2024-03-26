package frc.robot.commands.auto;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

public class LookAtTarget extends Command {
    private final SwerveSubsystem swerve;
    double kP = 0.3;
    double kI = 0.0005;
    double kD = 0.0005;

    private PIDController rotationPID = new PIDController(kP, kI, kD);

    public LookAtTarget(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        String limelightName = "limelight";

        SwerveDrive swerveDrive = swerve.getSwerveDrive();
        SwerveController swerveController = swerve.getSwerveController();

        if (LimelightHelpers.getTV(limelightName)) {
            double targetAngle = Math
                    .toDegrees(LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getRotation().getAngle());
            double rotSpeed = rotationPID.calculate(swerve.getHeading().getDegrees(), targetAngle);

            swerve.driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(0, 0,
                            Math.toRadians(targetAngle),
                            swerve.getHeading().getRadians(), swerveDrive.getMaximumVelocity()));

            // swerve.drive(new Translation2d(0, 0), MathUtil.clamp(rotSpeed, -1, 1),
            // false);
        } else {
            swerve.drive(new Translation2d(0, 0), 0, false); // force stop
        }
    }

    // @Override
    // public boolean isFinished() {
    // }

    @Override
    public void end(boolean interrupted) {
    }
}
