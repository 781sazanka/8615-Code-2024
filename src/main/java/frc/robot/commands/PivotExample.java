// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.Pivot;

import java.util.function.Supplier;

public class PivotExample extends Command {
    private final Pivot Pivot;

    private final double leftAxis;
    private final double rightAxis;

    public final boolean setSetpointEnabled = true;

    public PivotExample(
            Pivot subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue) {

        Pivot = subsystem;
        leftAxis = leftAxisValue.get();
        rightAxis = leftAxisValue.get();
        addRequirements(Pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (rightAxis >= 0.0) {
            Pivot.up(rightAxis);
        } else if (leftAxis >= 0.0) {
            Pivot.down(leftAxis);
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