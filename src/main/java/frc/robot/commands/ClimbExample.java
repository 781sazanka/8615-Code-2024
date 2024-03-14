// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

import java.util.function.Supplier;

public class ClimbExample extends Command {
    private final Climb climb;

    private final boolean isAButtonPressed;
    private final boolean isBButtonPressed;

    public final boolean setSetpointEnabled = true;

    public ClimbExample(
            Climb subsystem,
            Supplier<Boolean> AButton,
            Supplier<Boolean> BButton) {

        climb = subsystem;
        isAButtonPressed = AButton.get();
        isBButtonPressed = BButton.get();

        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isAButtonPressed) {
            climb.tighten();
        } else if (isBButtonPressed) {
            climb.loosen();
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