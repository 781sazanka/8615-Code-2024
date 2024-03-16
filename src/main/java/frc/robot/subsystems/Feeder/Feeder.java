// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    VictorSPX leaderMotor = new VictorSPX(Constants.Intake.leaderCANID);
    VictorSPX followerMotor = new VictorSPX(Constants.Intake.followerCANID);

    public Feeder() {
    }

    public void run(double output) {
        ControlMode mode = com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
        leaderMotor.set(mode, output);
        followerMotor.set(mode, output);
    }

    public Command feed(double output) {
        return startEnd(() -> run(output), () -> run(0));
    }
}
