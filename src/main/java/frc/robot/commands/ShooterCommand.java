// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Shooter;

public class ShooterCommand extends Command {
    private final boolean isAButtonPressed;
    private final boolean isBButtonPressed;
    // private final boolean isXButtonPressed;
    // private final boolean isYButtonPressed;

    private final double leftAxis;
    private final double rightAxis;

    private final Shooter shooter;

    public ShooterCommand(
            Shooter subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue,
            Supplier<Boolean> AButton,
            Supplier<Boolean> BButton,
            Supplier<Boolean> XButton,
            Supplier<Boolean> YButton) {
        shooter = subsystem;
        isAButtonPressed = AButton.get();
        isBButtonPressed = BButton.get();
        // isXButtonPressed = XButton.get();
        // isYButtonPressed = YButton.get();
        leftAxis = leftAxisValue.get();
        rightAxis = rightAxisValue.get();
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (leftAxis >= 0.0) {
            shooter.feed(leftAxis * Constants.Shooter.falconSpeedMultiplier);
        } else if (rightAxis >= 0.0) {
            shooter.shoot(rightAxis * Constants.Shooter.falconSpeedMultiplier);
        } else if (isAButtonPressed) {
            shooter.shoot(Constants.Shooter.falconMotorLowOutput);
        } else if (isBButtonPressed) {
            shooter.shoot(Constants.Shooter.falconMotorLowOutput);
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