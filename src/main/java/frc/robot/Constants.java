// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
  public static class Controller {
    public static final int controllerXboxID = 0;
    public static final int driveXboxID = 1;
    public static final int shooterXboxLeftAxisId = 1;
    public static final int shooterXboxRightAxisId = 5;
    public static final int pivotXboxUpAxisId = 2;
    public static final int pivotXboxDownAxisId = 6;
  }

  public static class CommandStatus {
    public static final boolean testShooter = true;
    public static final boolean testClimb = true;
    public static final boolean testPivot = true;
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class Shooter {
    public static final int topFalconMotorCANID = 1;
    public static final int bottomFalconMotorCANID = 4;
    public static final int sparkMaxCANID = 9;
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
    public static final int leaderCANID = 5;
    public static final int followerCANID = 6;

    public static final double loosenSpeed = -0.3;
    public static final double tightenSpeed = 0.4;
    public static final double keepCurrentPositionSpeed = 0.2;

    public static final int rotations = 15;
  }

  public static class Pivot {
    public static final int leaderCANID = 7;
    public static final int followerCANID = 8;

    public static final int lowerPosition = 15;
    public static final int higherPosition = 25;
  }

  public static class Feeder {
    public static final int leaderCANID = 40;
    public static final int followerCANID = 41;
  }
}
