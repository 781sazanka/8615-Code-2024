// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    final TalonFX motorLeader = new TalonFX(4, "rio");
    final TalonFX motorFollower = new TalonFX(1, "rio");

    final CANSparkMax sparkMaxFeederMotor = new CANSparkMax(51, MotorType.kBrushless);
    final CANSparkMax sparkMaxIntakeMotor = new CANSparkMax(54, MotorType.kBrushless);
    final CANSparkMax sparkMaxIntakeFeederMotor = new CANSparkMax(52, MotorType.kBrushless);
    final DigitalInput sensorInput = new DigitalInput(7);

    final ControlMode mode = com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
    int count = 0;
    boolean feederCleared = false;
    int getSensorCount = 0;
    boolean feederLoaded = false;
    int topSpeedCounter = 0;
    boolean movingMode = false;
    boolean stopState = false;

    public Shooter() {
        sparkMaxFeederMotor.restoreFactoryDefaults();
        sparkMaxIntakeMotor.restoreFactoryDefaults();
        sparkMaxIntakeFeederMotor.restoreFactoryDefaults();

        motorLeader.clearStickyFaults(0);
        motorFollower.clearStickyFaults(0);

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.25; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        motorLeader.getConfigurator().apply(slot0Configs);
        motorFollower.getConfigurator().apply(slot0Configs);
    }

    public void stop() {
        motorLeader.stopMotor();
        motorFollower.stopMotor();
        sparkMaxFeederMotor.stopMotor();
        sparkMaxIntakeMotor.stopMotor();
        sparkMaxIntakeFeederMotor.stopMotor();
    }

    public void runShooterMotor(double velocity) {
        // create a velocity closed-loop request, voltage output, slot 0 configs
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        motorLeader.setControl(m_request.withVelocity(velocity).withFeedForward(0.5));
        motorFollower.setControl(m_request.withVelocity(velocity).withFeedForward(0.5));
    }

    public void runFeederMotor(double output) {
        sparkMaxFeederMotor.set(output);
    }

    public void runIntakeMotor(double sparkMaxOutput, double sparkMaxIntakeFeederOutput) {
        sparkMaxIntakeMotor.set(sparkMaxOutput);
        sparkMaxIntakeFeederMotor.set(sparkMaxIntakeFeederOutput);
    }

    public void putData() {
        SmartDashboard.putNumber("[Shooter] top motor speed", motorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Shooter] bottom motor speed", motorFollower.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Shooter] intake NEO motor speed", sparkMaxIntakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("[Shooter] feeder motor speed", sparkMaxFeederMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("[Shooter] shooter is clear", isNoteInFeeder());
        SmartDashboard.putBoolean("[Shooter]", sensorInput.get());
        System.out.println(sensorInput.get());
    }

    public void shoot(double desiredShooterVelocity, double feederOutput) {
        double acceptableVelocityTolerance = 10;

        // runShooterMotor(desiredShooterVelocity * 0.6);
        // if (getShooterVelocity() + acceptableVelocityTolerance >=
        // desiredShooterVelocity * 0.6) {
        runShooterMotor(desiredShooterVelocity);
        if (getShooterVelocity() + acceptableVelocityTolerance >= desiredShooterVelocity) {

            // if (topSpeedCounter >= 5) {
            runFeederMotor(feederOutput);
            // }
            // }

        }

        feederLoaded = false;

    }

    public void getNote(double feederOutput, double intakeSparkMaxOutput, double intakeFeederSparkMaxOutput) {
        // boolean isNoteInFeeder = isNoteInFeeder();

        // if (isNoteInFeeder == false) {
        // if (stopState == false) {
        runFeederMotor(feederOutput);
        // }
        sparkMaxIntakeMotor.set(intakeSparkMaxOutput);
        sparkMaxIntakeFeederMotor.set(intakeFeederSparkMaxOutput);
        // } else {
        // stop();
        // }
    }

    public void moveNoteToFeeder() {
        if (sensorInput.get() == true) {
            runFeederMotor(0.2);
        } else {
            stop();
        }
    }

    public Command shootNoteCommand(double desiredShooterVelocity, double feederOutput) {
        return run(() -> shoot(desiredShooterVelocity, feederOutput)).withTimeout(1.5);
    }

    public Command getNoteCommand(double feederOutput, double intakeSparkMaxOutput, double intakeFeederSparkMaxOutput) {
        return run(() -> getNote(feederOutput, intakeSparkMaxOutput, intakeFeederSparkMaxOutput));
    }

    public double getShooterVelocity() {
        return motorLeader.getVelocity().getValueAsDouble();
    }

    public boolean isNoteInFeeder() {
        return sensorInput.get();
    }

    public boolean hasNoteShot() {
        return feederCleared;
    }

    @Override
    public void periodic() {
        putData();

        if (feederLoaded == false) {
            count = 0;
        }

        if (sensorInput.get() == false && count == 0) {
            feederLoaded = true;
            count += 1;
        }

        if (sensorInput.get() == true && feederLoaded == true) {
            movingMode = true;
        }

        if (sensorInput.get() == false && movingMode == true) {
            stopState = true;

        }

        SmartDashboard.putBoolean("feeder loaded", feederLoaded);

        if (getShooterVelocity() + 2 >= 50) {
            topSpeedCounter += 1;
        } else {
            topSpeedCounter = 0;
        }
    }
}
