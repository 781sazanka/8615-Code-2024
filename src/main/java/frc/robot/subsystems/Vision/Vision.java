package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Vector;

public class Vision {

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

    public Pose2d getEstimatedRoboPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    }

    public Matrix<N3, N1> getStandardDeviations() { // Borrowed all this from 3161 :]
        LimelightHelpers.PoseEstimate limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        Vector<N3> singleTagStD = VecBuilder.fill(2, 2, 4);
        // Get tag count and average position...
        int tagCount = limelightPose.tagCount;
        double avgDist = limelightPose.avgTagDist;

        SmartDashboard.putNumber("Vision avgDist", avgDist);
        SmartDashboard.putNumber("tagCount", tagCount);

        if (tagCount == 1 && avgDist > 4) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            SmartDashboard.putNumberArray("Vision STD",
                    singleTagStD.times(1 + (Math.pow(avgDist, 2) / 30)).getData());
            return singleTagStD.times(1 + (Math.pow(avgDist, 2) / 30));
        }
    }

    public double getTimestamp() {
        return Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("limelight") / 1000
                - LimelightHelpers.getLatency_Pipeline("limelight") / 1000;
    }

    public boolean getTV() {
        return LimelightHelpers.getTV("limelight");
    }
}
