package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class AutoCommand {
    static Shooter shooter;
    static Pivot pivot;

    public AutoCommand(Shooter shooter) {
        AutoCommand.shooter = shooter;
        // AutoCommand.pivot = pivot;

    }

    public Command intake() {
        return Commands.sequence(shooter.getNoteCommand(0.2, -0.9, -0.3).until(() -> !shooter.isNoteInFeeder()));
    }

    public Command score() {
        return Commands.run(() -> shooter.shoot(40, -0.7), shooter).withTimeout(1);
    }

    public Command scoreInOneSec() {
        return new RunCommand(() -> shooter.shoot(0, 0), shooter);
    }

    public Command rotatePivot(double angle) {
        return new RunCommand(() -> pivot.setPosition(angle), pivot);
    }

    // public Command rotatePivotInDegrees(double angle) {
    // Command returnObject = new RunCommand(() ->
    // pivot.setPosition(pivot.getCurrentPosition()), pivot);
    // String limelightName = "limelight";
    // if (LimelightHelpers.getTV(limelightName)) {
    // LimelightHelpers.setPriorityTagID(limelightName, 4);
    // LimelightHelpers.setPriorityTagID(limelightName, 7);
    // double targetAngle =
    // LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getY();
    // returnObject = new RunCommand(() -> pivot.setPositionFromDegrees(angle),
    // pivot);
    // }
    // return returnObject;
    // }

}
