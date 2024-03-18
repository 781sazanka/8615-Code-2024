// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    final TalonFX motorLeader = new TalonFX(Constants.Shooter.topFalconMotorCANID, "rio");
    final TalonFX motorFollower = new TalonFX(Constants.Shooter.bottomFalconMotorCANID, "rio");
    final CANSparkMax sparkMaxMotor = new CANSparkMax(Constants.Shooter.sparkMaxCANID, MotorType.kBrushless);
    VictorSPX redlineMotor = new VictorSPX(Constants.Feeder.motorCANID);

    final DigitalInput input = new DigitalInput(Constants.Shooter.photoSwitchSensorChannel);

    final Slot0Configs talonFXConfig = new Slot0Configs();

    public Shooter() {
        sparkMaxMotor.restoreFactoryDefaults();
        motorLeader.clearStickyFaults(0);
        motorFollower.clearStickyFaults(0);

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        var slot1Configs = new Slot0Configs();
        slot1Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot1Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0; // no output for error derivative

        motorLeader.getConfigurator().apply(slot0Configs);
        motorFollower.getConfigurator().apply(slot1Configs);
    }

    public void runShooterMotor(double velocity) {
        // create a velocity closed-loop request, voltage output, slot 0 configs
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

        motorLeader.setControl(m_request.withVelocity(velocity).withFeedForward(0.5));
        motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));
    }

    public void runFeederMotor(double output) {
        ControlMode mode = com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
        redlineMotor.set(mode, output);
    }

    public Command shoot(double output) {
        return startEnd(() -> runShooterMotor(output), () -> runShooterMotor(0));
    }

    public void putData() {

        SmartDashboard.putNumber("[Shooter] top motor speed", motorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Shooter] bottom motor speed", motorFollower.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        motorLeader.stopMotor();
        motorFollower.stopMotor();
        sparkMaxMotor.stopMotor();
        SmartDashboard.putBoolean("switch value", input.get());
    }
}
