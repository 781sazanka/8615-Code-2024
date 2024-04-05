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
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterExample;
import frc.robot.commands.auto.AutoCommand;
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
        private final Climb Climb = new Climb();
        private final Pivot Pivot = new Pivot();
        private final Camera cam = new Camera();
        // private final SwerveDrive swerveDrive = drivebase.getSwerveDrive();
        private final ShootToSpeaker shootToSpeaker = new ShootToSpeaker();
        private final Vision vision = new Vision();

        private final SendableChooser<Command> autoChooser;

        // private final AutoCommand autoShoot = new AutoCommand(Shooter);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // NamedCommands.registerCommand("getNote", autoShoot.intake());
                // NamedCommands.registerCommand("Shoot", autoShoot.score());
                // NamedCommands.registerCommand("Intake", autoShoot.intake());
                // NamedCommands.registerCommand("lookAtTarget", new LookAtTarget(drivebase));
                // NamedCommands.registerCommand("shootNote2", new autoShoot.score());

                Shooter.setDefaultCommand(
                                new ShooterCommand(Shooter,
                                                () -> controllerXbox.x().getAsBoolean(), // speaker shoot
                                                () -> controllerXbox.y().getAsBoolean(), // amp shoot
                                                () -> controllerXbox.a().getAsBoolean(), // intake
                                                () -> controllerXbox.b().getAsBoolean())); // intake reverse

                // Climb.setDefaultCommand(
                // new ClimbExample(Climb,
                // () -> controllerXbox.rightBumper().getAsBoolean(),
                // () -> controllerXbox.leftBumper().getAsBoolean(),
                // () -> controllerXbox.povRight().getAsBoolean(),
                // () -> controllerXbox.povLeft().getAsBoolean()));

                // Pivot.setDefaultCommand(
                // new PivotCommand(Pivot,
                // () -> driveXbox.rightBumper().getAsBoolean(),
                // () -> driveXbox.leftBumper().getAsBoolean(),
                // () -> driveXbox.rightTrigger().getAsBoolean(),
                // () -> driveXbox.button(1000).getAsBoolean()));

                cam.cameraStream();

                Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                                () -> -1 * MathUtil.applyDeadband(driveXbox.getLeftY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> -1 * MathUtil.applyDeadband(driveXbox.getLeftX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> -1 * driveXbox.getRightX() * 0.7);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
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
                // driveXbox.a().whileTrue(new LookAtTarget(drivebase));
                // debugJoystick.button(2).whileTrue(new AutoCommand(Shooter,
                // Pivot).rotatePivotInDegrees(45));

                // controllerXbox.a().whileTrue(Shooter.runRedlineMotor(0.1));
                // controllerXbox.b().whileTrue(Shooter.runRedlineMotor(0.3));
                // controllerXbox.x().whileTrue(Shooter.runRedlineMotor(0.7));
                // controllerXbox.y().whileTrue(Shooter.runRedlineMotor(0.9));

                // neo -0.35
                // redline -0.7
                // driveXbox.rightBumper().whileTrue(drivebase.rotateDriveBaseToSpeakerCommand());
                driveXbox.leftTrigger().onTrue(drivebase.zeroGyroCommand());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                // return drivebase.getAutonomousCommand("Preload Auto");
                return autoChooser.getSelected();
                // return null;
        }
}
