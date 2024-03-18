// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.Pivot;
import java.util.function.Supplier;

public class PivotExample extends Command {
    private final Pivot pivot;

    private final Supplier<Double> leftAxis;
    private final Supplier<Double> rightAxis;

    public PivotExample(
            Pivot subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue) {

        pivot = subsystem;
        leftAxis = leftAxisValue;
        rightAxis = rightAxisValue;
        addRequirements(pivot);
    }

    // Called when the command is initially scheduled.

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (rightAxis.get() >= 0.0) {
            pivot.setPosition(rightAxis.get() * 3 + pivot.currentPosition());
        } else if (leftAxis.get() >= 0.0) {
            pivot.setPosition(-1 * leftAxis.get() * 3 + pivot.currentPosition());
        }
    }

    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}