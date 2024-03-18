// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    final CANSparkMax leaderMotor = new CANSparkMax(Constants.Climb.leaderCANID, MotorType.kBrushless);
    final CANSparkMax followerMotor = new CANSparkMax(Constants.Climb.followerCANID, MotorType.kBrushless);
    private RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
    private RelativeEncoder followerEencoder = followerMotor.getEncoder();

    public Climb() {
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor, true);
    }

    public void putPosition() {
        SmartDashboard.putNumber("[Climb] follower position", followerEencoder.getPosition());
        SmartDashboard.putNumber("[Climb] leader position", leaderEncoder.getPosition());
    }

    public void set(double output) {
        leaderMotor.set(output);
    }
}