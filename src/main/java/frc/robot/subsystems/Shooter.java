// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  final TalonFX motor1 = new TalonFX(Constants.Shooter.topFalconMotorCanId, "rio");
  final TalonFX motor2 = new TalonFX(Constants.Shooter.bottomFalconMotorCanId, "rio");
  CANSparkMax SparkMaxMotor = new CANSparkMax(Constants.Shooter.sparkMaxCanId, MotorType.kBrushless);

  final DigitalInput input = new DigitalInput(Constants.Shooter.photoSwitchSensorChannel);;

  public Shooter() {
    SparkMaxMotor.restoreFactoryDefaults();
  }

  public void shoot(double output) {
    motor1.setControl(new DutyCycleOut(output));
    motor2.setControl(new DutyCycleOut(output));
  }

  public void feed(double output) {
    SparkMaxMotor.set(output);
  }

  @Override
  public void periodic() {
    motor1.stopMotor();
    motor2.stopMotor();
    SparkMaxMotor.stopMotor();
    SmartDashboard.putBoolean("switch value", input.get());
  }
}
