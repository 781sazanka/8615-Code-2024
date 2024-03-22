// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

    final CANSparkMax leaderMotor = new CANSparkMax(Constants.Climb.leaderCANID, MotorType.kBrushless);
    final CANSparkMax followerMotor = new CANSparkMax(Constants.Climb.followerCANID, MotorType.kBrushless);
    private RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
    private RelativeEncoder followerEencoder = followerMotor.getEncoder();

    public Climb() {
        leaderMotor.restoreFactoryDefaults();
        leaderMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor, true);
    }

    public void putPosition() {
        SmartDashboard.putNumber("[Climb] leader position", leaderEncoder.getPosition());
        SmartDashboard.putNumber("[Climb] follower position", followerEencoder.getPosition());
    }

    public void set(double output) {
        leaderMotor.set(output);
    }

    public void setWithEncoderPositionReset(double output, boolean resetEncoderPosition) {
        leaderMotor.set(output);
        if (resetEncoderPosition) {
            leaderEncoder.setPosition(0);
            followerEencoder.setPosition(0);
        }
    }

    public void climberUp(double output) {
        double topLeaderPosition = 0;
        double topFollowerPosition = 0;
        double offset = 0;
        if (leaderEncoder.getPosition() <= topLeaderPosition + offset) {
            set(output);
        } else {
            leaderMotor.stopMotor();
        }

        if (followerEencoder.getPosition() <= topFollowerPosition + offset) {
            set(output);
        } else {
            followerMotor.stopMotor();
        }
    }

    @Override
    public void periodic() {
        putPosition();
    }

    public Command moveToTheLowest() {
        return run(() -> setWithEncoderPositionReset(0.2, true)).withTimeout(2);
    }
}