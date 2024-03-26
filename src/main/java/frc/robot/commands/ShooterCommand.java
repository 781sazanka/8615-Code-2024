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
            double desiredShooterVelocity = 30;
            double acceptableVelocityTolerance = 0.5;
            double feederOutput = -0.5; // -1 to 1
            shooter.runShooterMotor(desiredShooterVelocity);
            if (shooter.getShooterVelocity() + acceptableVelocityTolerance >= desiredShooterVelocity)
                shooter.runFeederMotor(feederOutput);
        } else if (button2.get()) {
            double intakeNeoOutput = 0.5; // -1 to 1
            double intakeRedlineOutput = 0.5; // -1 to 1
            double feederOutput = 0.3; // -1 to 1

            // if (shooter.isNoteInFeeder() == false) {
            // shooter.runIntakeMotor(intakeNeoOutput, intakeRedlineOutput);
            // shooter.runFeederMotor(feederOutput);
            // } else {
            // shooter.stop();
            // }

            shooter.runIntakeMotor(intakeNeoOutput, intakeRedlineOutput);
            shooter.runFeederMotor(feederOutput);
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