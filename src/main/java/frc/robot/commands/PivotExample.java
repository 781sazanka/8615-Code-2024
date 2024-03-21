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

    private final Supplier<Double> leftAxis;
    private final Supplier<Double> rightAxis;

    private final Pivot pivot;

    public PivotExample(
            Pivot subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue,
            Supplier<Boolean> AButton,
            Supplier<Boolean> BButton) {
        pivot = subsystem;
        isAButtonPressed = AButton;
        isBButtonPressed = BButton;
        // isXButtonPressed = XButton.get();
        // isYButtonPressed = YButton.get();
        leftAxis = leftAxisValue;
        rightAxis = rightAxisValue;

        addRequirements(pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double position = -100000;

        pivot.putData();

        if (leftAxis.get() >= 0.1) {
            pivot.runMootr(leftAxis.get() * 0.5);
            position = pivot.currentPosition();
            // currentPosition = pivot.currentPosition();
        } else if (leftAxis.get() <= -0.1) {
            pivot.runMootr(leftAxis.get() * 0.5);
            position = pivot.currentPosition();
            // currentPosition = pivot.currentPosition();
        } else if (rightAxis.get() >= 0.1) {
        } else if (isAButtonPressed.get()) {
            // emergency stop button
            pivot.stop();

        } else if (isBButtonPressed.get()) {
        } else {
            // pivot.setPosition(pivot.currentPosition());
            pivot.stop();
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