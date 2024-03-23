// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class PivotExample extends Command {
    private final Supplier<Boolean> isAButtonPressed;
    private final Supplier<Boolean> isBButtonPressed;

    private final Supplier<Boolean> xButton;
    private final Supplier<Boolean> yButton;

    private final Pivot pivot;
    private boolean buttonEverPressed;
    double position;

    public PivotExample(
            Pivot subsystem,
            Supplier<Boolean> leftAxisValue,
            Supplier<Boolean> rightAxisValue,
            Supplier<Boolean> AButton,
            Supplier<Boolean> BButton) {
        pivot = subsystem;
        isAButtonPressed = AButton;
        isBButtonPressed = BButton;
        // isXButtonPressed = XButton.get();
        // isYButtonPressed = YButton.get();
        xButton = leftAxisValue;
        yButton = rightAxisValue;

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

        if (xButton.get()) {
            pivot.runMootr(-0.2);
            position = pivot.currentPosition();
            buttonEverPressed = true;

            // currentPosition = pivot.currentPosition();
        } else if (yButton.get()) {
            pivot.runMootr(0.2);
            position = pivot.currentPosition();
            buttonEverPressed = true;
        } else if (isBButtonPressed.get()) {
        } else {
            // pivot.setPosition(pivot.currentPosition());
            if (buttonEverPressed) {
                // pivot.setPosition(position);
                // pivot.stop();
                pivot.setPosition(position);
            } else {
                pivot.stop();
            }
        }

        SmartDashboard.putNumber("[Pivot] tx", LimelightHelpers.getTX("limelight"));
        SmartDashboard.putNumber("[Pivot] ty", LimelightHelpers.getTY("limelight"));
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