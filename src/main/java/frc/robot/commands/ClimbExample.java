// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;

import java.util.function.Supplier;

public class ClimbExample extends Command {
    private final Climb climb;

    private final Supplier<Boolean> isUpButtonPressed;
    private final Supplier<Boolean> isDownButtonPressed;

    public final boolean setSetpointEnabled = true;

    public ClimbExample(
            Climb subsystem,
            Supplier<Boolean> UpButton,
            Supplier<Boolean> DownButton) {

        climb = subsystem;
        isUpButtonPressed = UpButton;
        isDownButtonPressed = DownButton;

        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // climb.moveToTheLowest();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isUpButtonPressed.get()) {
            climb.set(-0.2);
        } else if (isDownButtonPressed.get()) {
            climb.set(0.2);
        }

        climb.putPosition();
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