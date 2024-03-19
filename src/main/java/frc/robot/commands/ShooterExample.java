// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.Shooter.Shooter;

public class ShooterExample extends Command {
    private final Supplier<Boolean> isAButtonPressed;
    private final Supplier<Boolean> isBButtonPressed;

    private final Supplier<Double> leftAxis;
    private final Supplier<Double> rightAxis;

    private final Shooter shooter;

    public ShooterExample(
            Shooter subsystem,
            Supplier<Double> leftAxisValue,
            Supplier<Double> rightAxisValue,
            Supplier<Boolean> AButton,
            Supplier<Boolean> BButton) {
        shooter = subsystem;
        isAButtonPressed = AButton;
        isBButtonPressed = BButton;
        // isXButtonPressed = XButton.get();
        // isYButtonPressed = YButton.get();
        leftAxis = leftAxisValue;
        rightAxis = rightAxisValue;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.putData();
        if (leftAxis.get() >= 0.1) {
        } else if (rightAxis.get() >= 0.1) {
            shooter.runShooterMotor(rightAxis.get() * 20);
        } else if (isAButtonPressed.get()) {
            // run feeder
            shooter.runFeederMotor(0.2);
        } else if (isBButtonPressed.get()) {
            // run shooter at very low speed
            shooter.runShooterMotor(1);
        } else {
            shooter.stop();
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