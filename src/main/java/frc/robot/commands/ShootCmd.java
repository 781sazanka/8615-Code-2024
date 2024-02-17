package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

public class ShootCmd extends Command {

    private final ShooterSubsystem shooter;
    Supplier<Boolean> hold_1, hold_2;

    public ShootCmd(ShooterSubsystem shooter, Supplier<Boolean> hold_1, Supplier<Boolean> hold_2) {
        this.shooter = shooter;
        this.hold_1 = hold_1;
        this.hold_2 = hold_2;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("ShootCmd Start!");
    }

    @Override
    public void execute() {
        if (hold_1.get()) {
            shooter.shoot();
        }
        if (hold_2.get()) {
            shooter.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ShootCmd End!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
