// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.Pivot.Pivot;

public class PivotCommand extends Command {
    private final Supplier<Boolean> button1;
    private final Supplier<Boolean> button2;
    private final Supplier<Boolean> button3;
    private final Supplier<Boolean> button4;
    private final Pivot pivot;
    private boolean buttonEverPressed;
    double position;

    public PivotCommand(
            Pivot subsystem,
            Supplier<Boolean> button1status,
            Supplier<Boolean> button2status,
            Supplier<Boolean> button3status,
            Supplier<Boolean> button4status) {
        pivot = subsystem;
        button1 = button1status;
        button2 = button2status;
        button3 = button3status;
        button4 = button4status;
        buttonEverPressed = false;
        addRequirements(pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivot.putData();

        if (button1.get()) {
            pivot.runMotor(-0.2);
            position = pivot.getCurrentPosition();
            buttonEverPressed = true;

            // currentPosition = pivot.currentPosition();
        } else if (button2.get()) {
            pivot.runMotor(0.2);
            position = pivot.getCurrentPosition();
            buttonEverPressed = true;

        } else if (button3.get()) {
            pivot.setPositionFromDegrees(60);
        }

        else {
            // pivot.setPosition(pivot.currentPosition());
            if (buttonEverPressed) {
                // pivot.setPosition(position);
                // pivot.stop();
                pivot.setPosition(position);
            } else {
                pivot.stop();
            }
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