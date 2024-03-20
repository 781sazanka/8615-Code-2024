// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    final TalonFX motorLeader = new TalonFX(30, "rio");
    final TalonFX motorFollower = new TalonFX(31, "rio");
    // final CANSparkMax sparkMaxMotor = new
    // CANSparkMax(Constants.Pivot.sparkMaxCANID, MotorType.kBrushless);
    final DutyCycleEncoder dutyCycleEncoder;

    public Pivot() {
        int dioChannel = 1;
        dutyCycleEncoder = new DutyCycleEncoder(dioChannel);

        // sparkMaxMotor.restoreFactoryDefaults();
        motorLeader.clearStickyFaults(0);
        motorFollower.clearStickyFaults(0);

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 24; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        motorLeader.getConfigurator().apply(slot0Configs);
        motorFollower.getConfigurator().apply(slot0Configs);
    }

    public void setPosition(double position) {
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // set position to 10 rotations
        motorLeader.setControl(m_request.withPosition(position));

        // motorFollower.setControl(new Follower(motorLeader.getDeviceID(), true));
    }

    public void runMootr(double output) {
        motorLeader.set(output);
    }

    public double currentPosition() {
        return motorLeader.getPosition().getValueAsDouble();
    }

    public void putData() {
        SmartDashboard.putNumber("[Pivot] top motor speed", motorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Pivot] bottom motor speed", motorFollower.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("[Pivot] top motor position", motorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("[Pivot] bottom motor position", motorFollower.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("[Pivot] encoder value", dutyCycleEncoder.getAbsolutePosition());
    }

    public void stop() {
        // sparkMaxMotor.stopMotor();
        motorLeader.set(0);
        motorFollower.set(0);
    }

    @Override
    public void periodic() {
        motorLeader.stopMotor();
        motorFollower.stopMotor();
        // sparkMaxMotor.stopMotor();
    }
}
