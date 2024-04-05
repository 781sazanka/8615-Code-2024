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
    private final Supplier<Boolean> isUpButtonPressed2;
    private final Supplier<Boolean> isDownButtonPressed2;

    public final boolean setSetpointEnabled = true;

    public ClimbExample(
            Climb subsystem,
            Supplier<Boolean> UpButton,
            Supplier<Boolean> DownButton, Supplier<Boolean> UpButton2,
            Supplier<Boolean> DownButton2) {

        climb = subsystem;
        isUpButtonPressed = UpButton;
        isDownButtonPressed = DownButton;

        isUpButtonPressed2 = UpButton2;
        isDownButtonPressed2 = DownButton2;

        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isUpButtonPressed.get()) {
            // climb.climberUp(0.2);
            climb.climberUp(0.36);
        } else if (isDownButtonPressed.get()) {
            climb.climberDown(0.36);
        } else if (isUpButtonPressed2.get()) {
            // climb.climberUp(0.2);
            // climb.set2(0.1);
            climb.climberUpWithoutLimit(0.12);
        } else if (isDownButtonPressed2.get()) {
            climb.climberDownWithoutLimit(0.12);
        } else {
            climb.stop();
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