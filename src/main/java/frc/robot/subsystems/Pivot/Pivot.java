// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    public final TalonFX motor1 = new TalonFX(Constants.Shooter.topFalconMotorCANID, "rio");

    public Pivot() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 24; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        motor1.getConfigurator().apply(slot0Configs);
    }

    public double currentPosition() {
        double currentPosition = motor1.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("[Pivot] current position", currentPosition);
        return currentPosition;
    }

    public void setPosition(double newPosition) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        motor1.setControl(m_request.withPosition(newPosition));
    }
}
