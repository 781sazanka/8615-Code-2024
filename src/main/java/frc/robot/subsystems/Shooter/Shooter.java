// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    final TalonFX motorLeader = new TalonFX(Constants.Shooter.topFalconMotorCanId, "rio");
    final TalonFX motorFollower = new TalonFX(Constants.Shooter.bottomFalconMotorCanId, "rio");
    final CANSparkMax sparkMaxMotor = new CANSparkMax(Constants.Shooter.sparkMaxCanId, MotorType.kBrushless);

    final DigitalInput input = new DigitalInput(Constants.Shooter.photoSwitchSensorChannel);

    final Slot0Configs talonFXConfig = new Slot0Configs();

    public Shooter() {
        sparkMaxMotor.restoreFactoryDefaults();
        talonFXConfig.kS = Constants.Shooter.kS; // Add 0.05 V output to overcome static friction
        talonFXConfig.kV = Constants.Shooter.kV; // A velocity target of 1 rps results in 0.12 V output
        talonFXConfig.kP = Constants.Shooter.kP; // An error of 1 rps results in 0.11 V output
        talonFXConfig.kI = Constants.Shooter.kI; // no output for integrated error
        talonFXConfig.kD = Constants.Shooter.kD; // no output for error derivative

        SmartDashboard.putNumber("kS", talonFXConfig.kS);
        SmartDashboard.putNumber("kV", talonFXConfig.kV);
        SmartDashboard.putNumber("kP", talonFXConfig.kP);
        SmartDashboard.putNumber("kI", talonFXConfig.kI);
        SmartDashboard.putNumber("kD", talonFXConfig.kD);

        motorLeader.clearStickyFaults(0);
        motorFollower.clearStickyFaults(0);

        talonFXConfig.kS = SmartDashboard.getNumber("kS", talonFXConfig.kS);
        talonFXConfig.kV = SmartDashboard.getNumber("kV", talonFXConfig.kV);
        talonFXConfig.kP = SmartDashboard.getNumber("kP", talonFXConfig.kP);
        talonFXConfig.kI = SmartDashboard.getNumber("kI", talonFXConfig.kI);
        talonFXConfig.kD = SmartDashboard.getNumber("kD", talonFXConfig.kD);
        motorLeader.getConfigurator().apply(talonFXConfig);
        motorFollower.getConfigurator().apply(talonFXConfig);
    }

    public void shoot(double output) {

        motorLeader.setControl(new DutyCycleOut(output));
        motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));

        SmartDashboard.putNumber("top motor speed", motorLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("bottom motor speed", motorFollower.getMotorVoltage().getValueAsDouble());
    }

    public void feed(double output) {
        sparkMaxMotor.set(output);
    }

    @Override
    public void periodic() {
        motorLeader.stopMotor();
        motorFollower.stopMotor();
        sparkMaxMotor.stopMotor();
        SmartDashboard.putBoolean("switch value", input.get());
    }
}
