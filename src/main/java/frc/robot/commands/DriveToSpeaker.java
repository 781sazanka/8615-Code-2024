package frc.robot.commands;

import java.util.logging.Logger;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.LL;

public class DriveToSpeaker extends Command {
    private SwerveSubsystem swerve;
    private LL LL;
    private DoubleSupplier translationX;
    private DoubleSupplier translationY;
    private double heading;

    Logger logger = Logger.getLogger(getClass().getName());

    public DriveToSpeaker(SwerveSubsystem swerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
        this.swerve = swerveSubsystem;
        this.translationX = translationX;
        this.translationY = translationY;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (LL.isTargetValid()) {
            heading = -1 * LL.getTxAsDouble() / 70;
        } else {
            logger.info("Lost target!!");
            heading = 0;
        }

        swerve.swerveDrive.drive(
                new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerve.swerveDrive.getMaximumVelocity(),
                        Math.pow(translationY.getAsDouble(), 3) * swerve.swerveDrive.getMaximumVelocity()),
                heading * swerve.swerveDrive.getMaximumAngularVelocity(),
                true,
                false);
    }
}
