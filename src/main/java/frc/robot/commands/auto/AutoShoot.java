package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoShoot extends Command {
    private final Shooter shooter;

    public AutoShoot(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        shooter.getNote(0, 0, 0);
        System.out.println("yeyayaya");
    }

    // @Override
    // public boolean isFinished() {
    // }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
