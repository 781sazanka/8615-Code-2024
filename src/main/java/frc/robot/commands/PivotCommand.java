// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class PivotCommand extends Command {
    private final Supplier<Boolean> button1;
    private final Supplier<Boolean> button2;
    private final Supplier<Boolean> button3;
    private final Supplier<Boolean> button4;
    private final Pivot pivot;
    public boolean buttonEverPressed = false;
    double position;
    boolean lastMovementWasUp = false;
    boolean lastMovementWasDown = false;

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
            pivot.up();
            position = pivot.getCurrentPosition();
            buttonEverPressed = true;
            lastMovementWasUp = true;
            // currentPosition = pivot.currentPosition();
        } else if (button2.get()) {
            pivot.down();
            position = pivot.getCurrentPosition();
            buttonEverPressed = true;
            lastMovementWasDown = true;
        } else if (button3.get()) {
            // if (LimelightHelpers.getTV("limelight")) {
            // double ty = LimelightHelpers.getTY("limelight");
            // double d1 = 1.78 - 0.1 / Math.tan(ty + 30);
            // double d2 = 0.07;
            // double h = 0.30;
            // double theta = (d1 + d2) / (1.78 - h);
            // double targetPosition = -(Math.toDegrees(Math.atan(theta)) - 38) * 40 / 52;
            // pivot.setPosition(targetPosition);
            // }
        } else {
            // pivot.setPosition(pivot.currentPosition());
            if (buttonEverPressed) {
                // pivot.setPosition(position);
                // pivot.stop();
                if (pivot.lowestAbsoluteEncoderValue <= pivot.getCurrentAbsoluteEncoderValue()) {
                    pivot.setPosition(position);
                } else {
                    pivot.stop();
                }
            } else {
                pivot.stop();
            }

            // pivot.getTargetAngle();
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