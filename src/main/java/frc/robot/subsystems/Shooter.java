// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  final TalonFX motorLeader = new TalonFX(Constants.Shooter.topFalconMotorCanId, "rio");
  final TalonFX motorFollower = new TalonFX(Constants.Shooter.bottomFalconMotorCanId, "rio");
  CANSparkMax SparkMaxMotor = new CANSparkMax(Constants.Shooter.sparkMaxCanId, MotorType.kBrushless);
  private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

  final DigitalInput input = new DigitalInput(Constants.Shooter.photoSwitchSensorChannel);;

  public Shooter() {
    SparkMaxMotor.restoreFactoryDefaults();
    talonFXConfig.Slot0.kP = Constants.Shooter.kP;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;
    talonFXConfig.Slot0.kS = 0;
    talonFXConfig.Slot0.kV = Constants.Shooter.kV;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorLeader.clearStickyFaults(0);
    motorFollower.clearStickyFaults(0);

    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      status = motorLeader.getConfigurator().apply(talonFXConfig);
      if (status.isOK())
        break;
    }

    if (!status.isOK()) {
      System.out.println(
          "Talon ID "
              + motorLeader.getDeviceID()
              + " failed config with error "
              + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motorFollower.getConfigurator().apply(talonFXConfig);
      if (status.isOK())
        break;
    }

    if (!status.isOK()) {
      System.out.println(
          "Talon ID "
              + motorFollower.getDeviceID()
              + " failed config with error "
              + status.toString());
    }
  }

  public void shoot(double output) {
    motorLeader.setControl(new DutyCycleOut(output));
    motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));
  }

  public void feed(double output) {
    SparkMaxMotor.set(output);
  }

  @Override
  public void periodic() {
    motorLeader.setNeutralMode(talonFXConfig.MotorOutput.NeutralMode);
    motorFollower.setNeutralMode(talonFXConfig.MotorOutput.NeutralMode);
    SparkMaxMotor.stopMotor();
    SmartDashboard.putBoolean("switch value", input.get());
  }
}
