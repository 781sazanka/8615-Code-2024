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

    final CANSparkMax leaderMotor = new CANSparkMax(58, MotorType.kBrushless);
    final CANSparkMax followerMotor = new CANSparkMax(57, MotorType.kBrushless);
    private RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
    private RelativeEncoder followerEncoder = followerMotor.getEncoder();
    private boolean reachedTheLowestPoint = false;
    private boolean leaderCannnotMove = false;
    private boolean followerCannotMove = false;

    public Climb() {
        leaderMotor.restoreFactoryDefaults();
        leaderMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.restoreFactoryDefaults();
        followerMotor.setIdleMode(IdleMode.kBrake);
        // followerMotor.follow(leaderMotor, true);
    }

    public void putPosition() {
        SmartDashboard.putNumber("[Climb] leader position", leaderEncoder.getPosition());
        SmartDashboard.putNumber("[Climb] follower position", followerEncoder.getPosition());
        // SmartDashboard.putBoolean("[Climb] reached the lowest",
        // reachedTheLowestPoint);
        SmartDashboard.putBoolean("[Climb] leader lowsest", leaderCannnotMove);
        SmartDashboard.putBoolean("[Climb] follower lowsest", followerCannotMove);
    }

    public void set(double output) {
        double pastPosition = leaderEncoder.getPosition();
        leaderMotor.set(output);
        double currentPosition = leaderEncoder.getPosition();
    }

    public void set1(double output) {
        double pastPosition = leaderEncoder.getPosition();
        leaderMotor.set(output);
        double currentPosition = leaderEncoder.getPosition();
    }

    public void set2(double output) {
        double pastPosition = leaderEncoder.getPosition();
        followerMotor.set(output);
        double currentPosition = leaderEncoder.getPosition();
    }

    public void setWithEncoderPositionReset(double output, boolean resetEncoderPosition) {
        leaderMotor.set(output);
        if (resetEncoderPosition) {
            leaderEncoder.setPosition(0);
            followerEncoder.setPosition(0);
        }
    }

    public void climberUp(double absoluteOutput) {
        // double topLeaderPosition = 0;
        // double topFollowerPosition = 0;
        // double offset = 0;
        // if (leaderEncoder.getPosition() <= topLeaderPosition + offset) {
        // set(output);
        // } else {
        // leaderMotor.stopMotor();
        // }

        // if (followerEencoder.getPosition() <= topFollowerPosition + offset) {
        // set(output);
        // } else {
        // followerMotor.stopMotor();
        // }
        if (leaderEncoder.getPosition() <= 47) {
            leaderMotor.set(absoluteOutput);
        }

        if (followerEncoder.getPosition() <= 47) {
            followerMotor.set(absoluteOutput);
        }
    }

    public void climberDown(double absoluteOutput) {
        if (leaderEncoder.getPosition() >= 1.5) {
            leaderMotor.set(-1 * absoluteOutput);
        }

        if (followerEncoder.getPosition() >= 1.5) {
            followerMotor.set(-1 * absoluteOutput);
        }
    }

    public void climberUpWithoutLimit(double absoluteOutput) {
        leaderMotor.set(absoluteOutput);
        followerMotor.set(absoluteOutput);
    }

    public void climberDownWithoutLimit(double absoluteOutput) {
        leaderMotor.set(-1 * absoluteOutput);
        followerMotor.set(-1 * absoluteOutput);
    }

    public void stop() {
        leaderMotor.set(0);
        followerMotor.set(0);
    }

    @Override
    public void periodic() {
        putPosition();
    }
}