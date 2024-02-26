// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CommandStatus;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Arm.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // private final FalconTest FalconTest = new FalconTest();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final Joystick exampleJoystick = new Joystick(0);
  private final XboxController exampleXbox = new XboxController(Constants.Controller.xboxId);
  private final Shooter Shooter = new Shooter();
  private final Climb Climb = new Climb();
  private final Arm Arm = new Arm();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (CommandStatus.testShooter) {
      Shooter.setDefaultCommand(
          new ShooterCommand(Shooter, () -> exampleXbox.getRawAxis(Constants.Shooter.topFalconMotorCanId),
              () -> exampleXbox.getRawAxis(Constants.Shooter.bottomFalconMotorCanId),
              () -> exampleXbox.getAButton(), () -> exampleXbox.getBButton(), () -> exampleXbox.getXButton(),
              () -> exampleXbox.getYButton()));
    }

    if (CommandStatus.testClimb) {
      Climb.setDefaultCommand(
          new ClimbCommand(Climb,
              () -> exampleXbox.getAButton(), () -> exampleXbox.getBButton()));
    }

    if (CommandStatus.testArm) {
      Arm.setDefaultCommand(
          new ArmCommand(Arm, () -> exampleXbox.getRawAxis(Constants.Shooter.topFalconMotorCanId),
              () -> exampleXbox.getRawAxis(Constants.Shooter.bottomFalconMotorCanId),
              () -> exampleXbox.getAButton(), () -> exampleXbox.getBButton()));
    }
    // Configure the trigger bindings

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
