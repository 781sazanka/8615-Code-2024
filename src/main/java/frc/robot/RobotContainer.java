// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Pivot.*;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.Vision.LL;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbExample;
import frc.robot.commands.DriveToSpeaker;
import frc.robot.commands.PivotExample;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.ShooterExample;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
        private final CommandJoystick debugJoystick = new CommandJoystick(3);
        private final CommandXboxController controllerXbox = new CommandXboxController(
                        Constants.Controller.controllerXboxID);
        private final CommandXboxController driveXbox = new CommandXboxController(Constants.Controller.driveXboxID);
        private final Shooter Shooter = new Shooter();
        // private final Climb Climb = new Climb();
        private final Pivot Pivot = new Pivot();
        private final Camera cam = new Camera();
        private final ShootToSpeaker shootToSpeaker = new ShootToSpeaker();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Shooter.setDefaultCommand(
                // new ShooterExample(Shooter, // () ->
                // // controllerXbox.getRawAxis(Constants.Controller.shooterXboxLeftAxisId),
                // // () ->
                // controllerXbox.getRawAxis(Constants.Controller.shooterXboxRightAxisId),
                // () -> debugJoystick.getRawAxis(0), () -> debugJoystick.getRawAxis(1),
                // () -> debugJoystick.getRawButton(1), () -> controllerXbox.getBButton()));
                Shooter.setDefaultCommand(
                                new ShooterExample(Shooter, () -> controllerXbox.getRawAxis(0),
                                                () -> controllerXbox.getRawAxis(1),
                                                () -> controllerXbox.a().getAsBoolean(),
                                                () -> controllerXbox.b().getAsBoolean()));

                // Climb.setDefaultCommand(
                // new ClimbExample(Climb,
                // () -> controllerXbox.x().getAsBoolean(),
                // () -> controllerXbox.y().getAsBoolean()));

                // Pivot.setDefaultCommand(
                // new PivotExample(Pivot, () -> controllerXbox.getRawAxis(0),
                // () -> controllerXbox.getRawAxis(1),
                // () -> controllerXbox.a().getAsBoolean(),
                // () -> controllerXbox.button(101).getAsBoolean()));

                cam.cameraStream();

                Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                                () -> MathUtil.applyDeadband(-1 * driveXbox.getLeftY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(-1 * driveXbox.getLeftX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> -1 * driveXbox.getRightX(),
                                () -> -1 * driveXbox.getRightY());
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
                // controllerXbox.rightBumper().whileTrue(new DriveToSpeaker(drivebase,
                // () -> MathUtil.applyDeadband(-1 * driveXbox.getLeftY(),
                // OperatorConstants.LEFT_Y_DEADBAND),
                // () -> MathUtil.applyDeadband(-1 * driveXbox.getLeftX(),
                // OperatorConstants.LEFT_X_DEADBAND)));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand("New Auto");
        }
}
