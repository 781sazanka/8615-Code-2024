// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

public class ArmCommand extends Command {
    private final Arm Arm;

    private final boolean isAButtonPressed;
    private final boolean isBButtonPressed;
    private final double leftAxis;
    private final double rightAxis;

    public final boolean setSetpointEnabled = true;

    public ArmCommand(
            Arm subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue,
            Supplier<Boolean> AButton,
            Supplier<Boolean> BButton) {

        Arm = subsystem;
        isAButtonPressed = AButton.get();
        isBButtonPressed = BButton.get();
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
            Arm.up(rightAxis * Constants.Shooter.falconSpeedMultiplier);
        } else if (leftAxis >= 0.0) {
            Arm.down(leftAxis * Constants.Shooter.falconSpeedMultiplier);
        } else if (isAButtonPressed) {
            Arm.setSetpoint(Constants.Arm.lowerPosition);
        } else if (isBButtonPressed) {
            Arm.setSetpoint(Constants.Arm.higherPosition);
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