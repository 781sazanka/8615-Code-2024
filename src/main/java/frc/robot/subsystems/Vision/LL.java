package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LL {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public double getDistance() {
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double limelightMountAngleDegrees = 25.0;
        double limelightLensHeightInches = 20.0;
        double goalHeightInches = 60.0;
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    public boolean isTargetValid() {
        return (table.getEntry("tv").getDouble(0.0) != 0);
    }

    public double getTxAsDouble() {
        return (table.getEntry("tx").getDouble(0.0));
    }
}
