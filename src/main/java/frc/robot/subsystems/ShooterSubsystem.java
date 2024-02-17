package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax intake_motor, top_shoot_motor, bottom_shoot_motor;
    private final SparkPIDController top_shoot_v_pid, bottom_shoot_v_pid;
    private final RelativeEncoder top_shoot_encoder, bottom_shoot_encoder;

    public ShooterSubsystem() {
        intake_motor = new CANSparkMax(Constants.Shooter.intake_motor_id, CANSparkLowLevel.MotorType.kBrushless);
        top_shoot_motor = new CANSparkMax(Constants.Shooter.top_shoot_motor_id, CANSparkLowLevel.MotorType.kBrushless);
        bottom_shoot_motor = new CANSparkMax(Constants.Shooter.bottom_shoot_motor_id, CANSparkLowLevel.MotorType.kBrushless);

        top_shoot_v_pid = top_shoot_motor.getPIDController();
        bottom_shoot_v_pid = bottom_shoot_motor.getPIDController();

        top_shoot_encoder = top_shoot_motor.getEncoder();
        bottom_shoot_encoder = bottom_shoot_motor.getEncoder();

        initialize_v_pid_controllers();
    }

    public void intake() {
        intake_motor.set(Constants.Shooter.intake_speed);
    }

    public void shoot() {
        top_shoot_motor.set(Constants.Shooter.shoot_speed);
        bottom_shoot_motor.set(Constants.Shooter.shoot_speed);
    }

    public void velocity_pid_shoot(double target_speed) {
        top_shoot_v_pid.setReference(target_speed, CANSparkBase.ControlType.kVelocity);
        bottom_shoot_v_pid.setReference(target_speed, CANSparkBase.ControlType.kVelocity);
    }

    public boolean is_target_velocity_reached(double target_speed, double deadband) {
        return is_motor_target_velocity_reached(top_shoot_encoder, target_speed, deadband) &&
                is_motor_target_velocity_reached(bottom_shoot_encoder, target_speed, deadband);
    }

    private void stop_all_motor() {
        intake_motor.set(0.0);
        top_shoot_motor.set(0.0);
        bottom_shoot_motor.set(0.0);
    }

    private boolean is_motor_target_velocity_reached(RelativeEncoder encoder, double target_speed, double deadband) {
        return Math.abs(encoder.getVelocity() - target_speed) < deadband;
    }

    private void initialize_v_pid_controllers() {
        set_pid(top_shoot_v_pid,
                Constants.Shooter.PID.kP_1, Constants.Shooter.PID.kI_1, Constants.Shooter.PID.kD_1);
        set_pid(bottom_shoot_v_pid,
                Constants.Shooter.PID.kP_2, Constants.Shooter.PID.kI_2, Constants.Shooter.PID.kD_2);
    }

    private void set_pid(SparkPIDController pid_controller, double p, double i, double d) {
        pid_controller.setP(p);
        pid_controller.setI(i);
        pid_controller.setD(d);
    }
    @Override
    public void periodic() {
        stop_all_motor();
    }
}
