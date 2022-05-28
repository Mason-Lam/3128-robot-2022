package frc.team3128.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.FieldConstants.HUB_RADIUS;

import java.util.function.DoubleSupplier;

import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Shooter;

public class CmdShootDist extends CommandBase {
    private Shooter shooter;
    //private LimelightSubsystem limelights;
    private Hood hood;
    private DoubleSupplier dist;
    public CmdShootDist(DoubleSupplier dist) {
        shooter = Shooter.getInstance();
        //this.limelights = limelights;
        hood = Hood.getInstance();
        this.dist = dist;
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        //limelights.turnShooterLEDOn();
    }
    
    @Override
    public void execute() {
        //double dist = limelights.calculateDistance("shooter");
        double distance = Units.metersToInches(dist.getAsDouble()) - Units.metersToInches(HUB_RADIUS);
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(distance));
        hood.startPID(hood.calculateAngleFromDistance(distance));
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        //limelights.turnShooterLEDOff();
        Log.info("CmdShootDist", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
