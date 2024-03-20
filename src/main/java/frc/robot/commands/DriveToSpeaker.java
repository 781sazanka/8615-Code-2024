package frc.robot.commands;

import java.util.logging.Logger;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.LL;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToSpeaker extends Command {
    private SwerveSubsystem swerve;
    private LL ll;
    private DoubleSupplier translationX;
    private DoubleSupplier translationY;

    Logger logger = Logger.getLogger(getClass().getName());

    public DriveToSpeaker(SwerveSubsystem swerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
        this.swerve = swerveSubsystem;
        this.translationX = translationX;
        this.translationY = translationY;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            LimelightHelpers.setPriorityTagID("limelight", 7); // Set to aim at blue speaker
        } else {
            LimelightHelpers.setPriorityTagID("limelight", 4); // Set to aim at red speaker
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double headingVelocity;
        System.out.println("right bumper pressed");
        SmartDashboard.putBoolean("valid target", ll.isTargetValid());
        if (ll.isTargetValid()) {
            headingVelocity = -1 * ll.getTxAsDouble() / 70;
        } else {
            logger.info("Lost target!!");
            headingVelocity = 0;
        }
        SmartDashboard.putNumber("heading velocity", headingVelocity);

        // swerve.swerveDrive.drive(
        // new Translation2d(Math.pow(translationX.getAsDouble(), 3) *
        // swerve.swerveDrive.getMaximumVelocity(),
        // Math.pow(translationY.getAsDouble(), 3) *
        // swerve.swerveDrive.getMaximumVelocity()),
        // headingVelocity * swerve.swerveDrive.getMaximumAngularVelocity(),
        // true,
        // false);
    }
}
