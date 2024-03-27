// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

    final TalonFX motorLeader = new TalonFX(30, "rio");
    final TalonFX motorFollower = new TalonFX(32, "rio");
    final DutyCycleEncoder dutyCycleEncoder;

    double maxEncoderValue = -2; // lowest position
    double minEncoderValue = -14; // highest position

    double kP = 0.3;
    double kI = 0.0005;
    double kD = 0.0005;

    private PIDController rotationPID = new PIDController(kP, kI, kD);

    public Pivot() {
        int dioChannel = 0;
        dutyCycleEncoder = new DutyCycleEncoder(dioChannel);

        // sparkMaxMotor.restoreFactoryDefaults();
        motorLeader.clearStickyFaults(0);
        motorFollower.clearStickyFaults(0);

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 4; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        motorLeader.getConfigurator().apply(slot0Configs);
        motorFollower.getConfigurator().apply(slot0Configs);
    }

    public void setPosition(double position) {
        if (minEncoderValue <= position && position <= maxEncoderValue) {
            final PositionVoltage m_request = new PositionVoltage(position).withSlot(0);
            motorLeader.setControl(m_request);
        } else {

            System.out.println("Error: Set pivot position out of range!");

            if (position < minEncoderValue) {
                final PositionVoltage m_request = new PositionVoltage(minEncoderValue).withSlot(0);
                motorLeader.setControl(m_request);
            } else if (maxEncoderValue < position) {
                final PositionVoltage m_request = new PositionVoltage(maxEncoderValue).withSlot(0);
                motorLeader.setControl(m_request);
            }
        }
    }

    public void setPositionFromDegrees(double targetPosition) {
        // safety feature
        double currentPosition = getCurrentPosition();
        if (currentPosition < minEncoderValue) {
            final PositionVoltage m_request = new PositionVoltage(minEncoderValue).withSlot(0);
            motorLeader.setControl(m_request);
        } else if (maxEncoderValue < currentPosition) {
            final PositionVoltage m_request = new PositionVoltage(maxEncoderValue).withSlot(0);
            motorLeader.setControl(m_request);
        } else {
            double currentAbsoluteEncoderPosition = dutyCycleEncoder.getAbsolutePosition() * 360;
            double output = rotationPID.calculate(currentAbsoluteEncoderPosition,
                    targetPosition);
            rotationPID.setTolerance(1.0);

            if (minEncoderValue <= currentPosition && currentPosition <= maxEncoderValue) {
                if (rotationPID.atSetpoint()) {
                    setPosition(getCurrentPosition());
                } else {
                    runMotor(MathUtil.clamp(output, -0.3, 0.3));
                }
            }
        }
    }

    public void runMotor(double output) {
        motorLeader.setControl(new DutyCycleOut(output));
    }

    public double getCurrentPosition() {
        return motorLeader.getPosition().getValueAsDouble();
    }

    public void putData() {
        SmartDashboard.putNumber("[Pivot] top motor speed", motorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Pivot] bottom motor speed", motorFollower.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Pivot] top motor position", motorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("[Pivot] bottom motor position", motorFollower.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("[Pivot] encoder", dutyCycleEncoder.getAbsolutePosition());
    }

    public void stop() {
        motorLeader.set(0);
        motorFollower.set(0);
    }

    @Override
    public void periodic() {
        putData();

        double currentPosition = getCurrentPosition();
    }
}
