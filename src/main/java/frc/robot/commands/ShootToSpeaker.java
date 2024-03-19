package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.LL;

public class ShootToSpeaker extends Command {
    private Shooter shooter;
    private LL LL;
    InterpolatingDoubleTreeMap dataMap = new InterpolatingDoubleTreeMap();

    public void Shoot() {
        addRequirements(shooter);
        // shooter.shoot(interpolatedOutput(LL.getDistance()));
    }

    public double interpolatedOutput(double distance) {
        dataMap.put(125.0, 450.0);
        dataMap.put(200.0, 510.0);
        dataMap.put(268.0, 525.0);
        dataMap.put(312.0, 550.0);
        dataMap.put(326.0, 650.0);

        return dataMap.get(distance);
    }
}
