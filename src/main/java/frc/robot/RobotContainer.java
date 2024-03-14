// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants.CommandStatus;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmExample;
import frc.robot.commands.ClimbExample;
import frc.robot.commands.DriveToSpeaker;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.ShooterExample;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Camera;

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
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final CommandXboxController controllerXbox = new CommandXboxController(Constants.Controller.controllerXboxID);
  private final CommandXboxController driveXbox = new CommandXboxController(Constants.Controller.driveXboxID);
  private final Shooter Shooter = new Shooter();
  private final Climb Climb = new Climb();
  private final Arm Arm = new Arm();
  private final Camera cam = new Camera();

  private final ShootToSpeaker shootToSpeaker = new ShootToSpeaker();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (CommandStatus.testShooter) {
      Shooter.setDefaultCommand(
          new ShooterExample(Shooter, () -> controllerXbox.getRawAxis(Constants.Shooter.topFalconMotorCANID),
              () -> controllerXbox.getRawAxis(Constants.Shooter.bottomFalconMotorCANID),
              () -> controllerXbox.a().getAsBoolean(), () -> controllerXbox.b().getAsBoolean()));
    }

    if (CommandStatus.testClimb) {
      Climb.setDefaultCommand(
          new ClimbExample(Climb,
              () -> controllerXbox.x().getAsBoolean(), () -> controllerXbox.y().getAsBoolean()));
    }

    if (CommandStatus.testArm) {
      Arm.setDefaultCommand(
          new ArmExample(Arm, () -> controllerXbox.getRawAxis(Constants.Controller.armXboxUpAxisId),
              () -> controllerXbox.getRawAxis(Constants.Controller.armXboxDownAxisId)));
    }

    cam.cameraStream();

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driveXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driveXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> driveXbox.getRightX(),
        () -> driveXbox.getRightY());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

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
    controllerXbox.leftBumper().onTrue(Commands.runOnce(() -> shootToSpeaker.Shoot()));
    controllerXbox.rightBumper().whileTrue(new DriveToSpeaker(drivebase,
        () -> MathUtil.applyDeadband(-controllerXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-controllerXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)));
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
