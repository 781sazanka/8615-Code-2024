package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor, topShootMotor, bottomShootMotor;
    private final SparkPIDController topShootVPID, bottomShootVPID;
    private final RelativeEncoder topShootEncoder, bottomShootEncoder;

    public ShooterSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Shooter.intake_motor_id,
            CANSparkLowLevel.MotorType.kBrushless);
        topShootMotor = new CANSparkMax(Constants.Shooter.top_shoot_motor_id,
            CANSparkLowLevel.MotorType.kBrushless);
        bottomShootMotor = new CANSparkMax(Constants.Shooter.bottom_shoot_motor_id,
            CANSparkLowLevel.MotorType.kBrushless);

        topShootVPID = topShootMotor.getPIDController();
        bottomShootVPID = bottomShootMotor.getPIDController();

        topShootEncoder = topShootMotor.getEncoder();
        bottomShootEncoder = bottomShootMotor.getEncoder();

        initializeVPIDControllers();
    }

    public void intake() {
        intakeMotor.set(Constants.Shooter.intake_speed);
    }

    public void shoot() {
        topShootMotor.set(Constants.Shooter.shoot_speed);
        bottomShootMotor.set(Constants.Shooter.shoot_speed);
    }

    public void velocityPIDShoot(double targetSpeed) {
        topShootVPID.setReference(targetSpeed, CANSparkBase.ControlType.kVelocity);
        bottomShootVPID.setReference(targetSpeed, CANSparkBase.ControlType.kVelocity);
    }

    public boolean isTargetVelocityReached(double targetSpeed, double deadband) {
        return isMotorTargetVelocityReached(topShootEncoder, targetSpeed, deadband) &&
            isMotorTargetVelocityReached(bottomShootEncoder, targetSpeed, deadband);
    }

    private void stopAllMotor() {
        intakeMotor.set(0.0);
        topShootMotor.set(0.0);
        bottomShootMotor.set(0.0);
    }

    private boolean isMotorTargetVelocityReached(RelativeEncoder encoder, double targetSpeed,
        double deadband) {
        return Math.abs(encoder.getVelocity() - targetSpeed) < deadband;
    }

    private void initializeVPIDControllers() {
        setPID(topShootVPID,
            Constants.Shooter.PID.kP_1, Constants.Shooter.PID.kI_1, Constants.Shooter.PID.kD_1);
        setPID(bottomShootVPID,
            Constants.Shooter.PID.kP_2, Constants.Shooter.PID.kI_2, Constants.Shooter.PID.kD_2);
    }

    private void setPID(SparkPIDController pidController, double p, double i, double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    @Override
    public void periodic() {
        stopAllMotor();
    }
}
