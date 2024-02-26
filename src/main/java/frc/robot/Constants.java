// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Controller {
    public static final int xboxId = 0;
    public static final int shooterXboxLeftAxisId = 1;
    public static final int shooterXboxRightAxisId = 5;
  }

  public static class CommandStatus {
    public static final boolean testShooter = false;
    public static final boolean testClimb = false;
    public static final boolean testArm = false;
  }

  public static class Shooter {
    public static final int topFalconMotorCanId = 1;
    public static final int bottomFalconMotorCanId = 4;
    public static final int sparkMaxCanId = 9;

    public static final double falconSpeedMultiplier = 0.5; // 0.0 - 1.0

    public static final double falconMotorLowOutput = 0.3; // 0.0 - 1.0
    public static final double falconMotorHighOutput = 0.5; // 0.0 - 1.0

    public static final int photoSwitchSensorChannel = 0;

    public static final double kS = 0.05; // Add 0.05 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kP = 0.11; // An error of 1 rps results in 0.11 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0; // no output for error derivative
  }

  public static class Climb {
    public static final int leaderCanId = 5;
    public static final int followerCanId = 6;

    public static final double loosenSpeed = 0.1;
    public static final double tightenSpeed = 0.1;

    public static final int rotations = 15;
  }

  public static class Arm {
    public static final int leaderCanId = 7;
    public static final int followerCanId = 8;

    public static final int lowerPosition = 15;
    public static final int higherPosition = 25;
  }
}
