package frc.team3128.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.FieldConstants.HUB_RADIUS;

import java.util.function.DoubleSupplier;

import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Shooter;

public class CmdShootContinue extends CommandBase {
    private Shooter shooter;
    //private LimelightSubsystem limelights;
    private Hood hood;
    private DoubleSupplier dist;
    public CmdShootContinue(Shooter shooter, Hood hood, DoubleSupplier dist) {
        this.shooter = shooter;
        //this.limelights = limelights;
        this.hood = hood;
        this.dist = dist;
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        double distance = Units.metersToInches(dist.getAsDouble()) - Units.metersToInches(HUB_RADIUS);
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(distance));
        hood.startPID(hood.calculateAngleFromDistance(distance));
    }
    
    @Override
    public void end(boolean interrupted) {
        Log.info("CmdShootDist", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}