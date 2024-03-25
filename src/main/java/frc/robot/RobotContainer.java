// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Pivot.*;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbExample;
import frc.robot.commands.DriveToSpeaker;
import frc.robot.commands.PivotExample;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.ShooterExample;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.LookAtTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

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
        private final SwerveDrive swerveDrive = drivebase.getSwerveDrive();

        private final SendableChooser<Command> autoChooser;

        private final AutoShoot autoShoot = new AutoShoot(Shooter);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                NamedCommands.registerCommand("getNotes", autoShoot.score());
                NamedCommands.registerCommand("lookAtTarget", new LookAtTarget(drivebase));

                Shooter.setDefaultCommand(
                                new ShooterExample(Shooter, () -> controllerXbox.getRawAxis(0),
                                                () -> controllerXbox.getRawAxis(1),
                                                () -> controllerXbox.x().getAsBoolean(),
                                                () -> controllerXbox.y().getAsBoolean()));

                // Climb.setDefaultCommand(
                // new ClimbExample(Climb,
                // () -> controllerXbox.x().getAsBoolean(),
                // () -> controllerXbox.y().getAsBoolean(),
                // () -> controllerXbox.a().getAsBoolean(),
                // () -> controllerXbox.b().getAsBoolean()));

                Pivot.setDefaultCommand(
                                new PivotExample(Pivot, () -> controllerXbox.rightBumper().getAsBoolean(),
                                                () -> controllerXbox.leftBumper().getAsBoolean(),
                                                () -> controllerXbox.a().getAsBoolean(),
                                                () -> controllerXbox.b().getAsBoolean()));

                cam.cameraStream();

                // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                // () -> MathUtil.applyDeadband(-1 * driveXbox.getLeftY(),
                // OperatorConstants.LEFT_Y_DEADBAND),
                // () -> MathUtil.applyDeadband(-1 * driveXbox.getLeftX(),
                // OperatorConstants.LEFT_X_DEADBAND),
                // () -> -1 * driveXbox.getRightX(),
                // () -> -1 * driveXbox.getRightY());

                Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                                () -> MathUtil.applyDeadband(driveXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driveXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                () -> debugJoystick.getRawAxis(1) * 0.5);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser("New Auto");
                SmartDashboard.putData("Auto Chooser", autoChooser);
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
                debugJoystick.button(1).whileTrue(new LookAtTarget(drivebase));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                // return drivebase.getAutonomousCommand("New Auto");
                return autoChooser.getSelected();
        }
}
