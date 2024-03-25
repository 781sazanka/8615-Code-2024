package frc.robot.commands.auto;

import com.ctre.phoenix6.sim.ChassisReference;

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
    double kP = 0.04;
    double kI = 0.0005;
    double kD = 0.005;

    private PIDController rotationPID = new PIDController(kP, kI, kD);

    public LookAtTarget(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        rotationPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        SwerveDrive swerveDrive = swerve.getSwerveDrive();

        double angleInDegrees = 180;
        swerve.driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(0, 0, Math.toRadians(angleInDegrees + 45),
                        swerve.getHeading().getRadians(), swerveDrive.getMaximumVelocity()));
    }

    // @Override
    // public boolean isFinished() {
    // }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}
