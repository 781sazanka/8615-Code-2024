package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoShoot {
    static Shooter shooter;

    public AutoShoot(Shooter shooter) {
        AutoShoot.shooter = shooter;
    }

    public Command score() {
        return Commands.sequence(shooter.getNoteCommand(0, 0, 0).withTimeout(2));
    }
}
