// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Shooter {
    public static final int topFalconMotorCanId = 1;
    public static final int bottomFalconMotorCanId = 4;
    public static final int sparkMaxCanId = 9;

    public static final double falconSpeedMultiplier = 0.5; // 0.0 - 1.0

    public static final double falconMotorLowOutput = 0.3; // 0.0 - 1.0
    public static final double falconMotorHighOutput = 0.5; // 0.0 - 1.0
  }
}
