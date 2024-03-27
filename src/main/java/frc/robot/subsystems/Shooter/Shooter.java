// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    final TalonFX motorLeader = new TalonFX(1, "rio");
    final TalonFX motorFollower = new TalonFX(4, "rio");

    // VictorSPX redlineMotor = new VictorSPX(40);
    final CANSparkMax sparkMaxFeederMotor = new CANSparkMax(50, MotorType.kBrushless);
    final CANSparkMax sparkMaxIntakeMotor = new CANSparkMax(41, MotorType.kBrushless);
    final VictorSPX redlineController = new VictorSPX(50);
    final DigitalInput sensorInput = new DigitalInput(5);

    final ControlMode mode = com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;

    public Shooter() {
        sparkMaxFeederMotor.restoreFactoryDefaults();
        sparkMaxIntakeMotor.restoreFactoryDefaults();

        motorLeader.clearStickyFaults(0);
        motorFollower.clearStickyFaults(0);

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
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
        redlineController.set(mode, 0);
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

    public void runIntakeMotor(double sparkMaxOutput, double redlineOutput) {
        sparkMaxIntakeMotor.set(sparkMaxOutput);
        redlineController.set(mode, redlineOutput);
    }

    public void putData() {
        SmartDashboard.putNumber("[Shooter] top motor speed", motorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Shooter] bottom motor speed", motorFollower.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Shooter] intake NEO motor speed", sparkMaxIntakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("[Shooter] feeder motor speed", sparkMaxFeederMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("[Shooter] intake redline motor speed", redlineController.getMotorOutputPercent());
        SmartDashboard.putBoolean("[Shooter] shooter is clear", isNoteInFeeder());
    }

    public void shoot(double desiredShooterVelocity, double feederOutput) {
        double acceptableVelocityTolerance = 0.5;
        runShooterMotor(desiredShooterVelocity);
        if (getShooterVelocity() + acceptableVelocityTolerance >= desiredShooterVelocity)
            runFeederMotor(feederOutput);
    }

    public void getNote(double feederOutput, double intakeSparkMaxOutput, double intakeRedlineOutput) {
        boolean isNoteInFeeder = isNoteInFeeder();
        if (isNoteInFeeder == false) {
            runFeederMotor(feederOutput);
            runIntakeMotor(intakeSparkMaxOutput, intakeRedlineOutput);
        } else {
            stop();
        }
    }

    public Command getNoteCommand(double feederOutput, double intakeSparkMaxOutput, double intakeRedlineOutput) {
        return run(() -> getNote(feederOutput, intakeSparkMaxOutput, intakeRedlineOutput));
    }

    public double getShooterVelocity() {
        return motorLeader.getVelocity().getValueAsDouble();
    }

    public boolean isNoteInFeeder() {
        return sensorInput.get();
    }

    @Override
    public void periodic() {
        putData();
    }
}
