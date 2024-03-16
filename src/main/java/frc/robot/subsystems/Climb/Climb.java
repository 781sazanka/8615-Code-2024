// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

    final CANSparkMax leaderMotor = new CANSparkMax(Constants.Climb.leaderCANID, MotorType.kBrushless);
    final CANSparkMax followerMotor = new CANSparkMax(Constants.Climb.followerCANID, MotorType.kBrushless);

    public Climb() {
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor);
    }

    public void loosen() {
        leaderMotor.set(Constants.Climb.keepCurrentPositionSpeed + Constants.Climb.loosenSpeed);
    }

    public void tighten() {
        leaderMotor.set(Constants.Climb.tightenSpeed);
    }

    public void setCurrent() {
        leaderMotor.set(Constants.Climb.keepCurrentPositionSpeed);
    }
}