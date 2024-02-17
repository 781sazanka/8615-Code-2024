// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final Joystick controller = new Joystick(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        shooter.setDefaultCommand(new ShootCmd(shooter,
                // X Button to Shoot, A Button to Intake
                () -> controller.getRawButton(0), () -> controller.getRawButton(1)
                )
        );

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
