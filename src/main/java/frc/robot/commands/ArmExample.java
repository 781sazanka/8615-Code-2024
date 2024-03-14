// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.arm.Arm;

public class ArmExample extends Command {
    private final Arm Arm;

    private final double leftAxis;
    private final double rightAxis;

    public final boolean setSetpointEnabled = true;

    public ArmExample(
            Arm subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue) {

        Arm = subsystem;
        leftAxis = leftAxisValue.get();
        rightAxis = leftAxisValue.get();
        addRequirements(Arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (rightAxis >= 0.0) {
            Arm.up(rightAxis);
        } else if (leftAxis >= 0.0) {
            Arm.down(leftAxis);
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