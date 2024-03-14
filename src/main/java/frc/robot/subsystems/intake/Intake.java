// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    VictorSP leaderMotor = new VictorSP(0);
    VictorSP followerMotor = new VictorSP(0);

    public Intake() {
    }

    public void run(double output) {
        leaderMotor.set(output);
        followerMotor.set(output);
    }

    public Command feed(double output) {
        return startEnd(() -> run(output), () -> run(0));
    }
}
