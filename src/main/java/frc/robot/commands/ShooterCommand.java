// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.Shooter.Shooter;;

public class ShooterCommand extends Command {
    private final Supplier<Boolean> button1;
    private final Supplier<Boolean> button2;
    private final Supplier<Boolean> button3;
    private final Supplier<Boolean> button4;
    private final Shooter shooter;

    public ShooterCommand(
            Shooter subsystem,
            Supplier<Boolean> button1status,
            Supplier<Boolean> button2status,
            Supplier<Boolean> button3status,
            Supplier<Boolean> button4status) {
        shooter = subsystem;
        button1 = button1status;
        button2 = button2status;
        button3 = button3status;
        button4 = button4status;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (button1.get()) {
            shooter.shoot(50, -0.8);
        }

        else if (button2.get()) {
            shooter.shoot(10, -0.3);
        }

        else if (button3.get()) {
            shooter.getNote(-0.8, -0.9, -0.3);
        }

        else if (button4.get()) {
            shooter.getNote(0.2, 0.9, 0.5);
        }

        else {
            shooter.stop(); // force stop
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}