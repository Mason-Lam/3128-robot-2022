package frc.team3128.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdInPlaceTurnOdom extends ProfiledPIDCommand {

    private NAR_Drivetrain drivetrain;
    private LimelightSubsystem limelights;

    public CmdInPlaceTurnOdom(NAR_Drivetrain drivetrain, LimelightSubsystem limelights) {
        super(
            new ProfiledPIDController(DriveConstants.TURN_kP, DriveConstants.TURN_kI, DriveConstants.TURN_kD,
            new TrapezoidProfile.Constraints(DriveConstants.MAX_DRIVE_VELOCITY,DriveConstants.MAX_DRIVE_ACCELERATION)),
            drivetrain::getHeading,
            drivetrain.calculateDesiredAngle(),
            (output,setpoint) -> drivetrain.tankDrive(output + Math.copySign(DriveConstants.TURN_kF, output), -output - Math.copySign(DriveConstants.TURN_kF, output)),
            drivetrain
        );

        this.drivetrain = drivetrain;
        this.limelights = limelights;

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(DriveConstants.TURN_TOLERANCE);
    }

    @Override
    public void initialize() {
        super.initialize();
        TrapezoidProfile.State setpoint = new TrapezoidProfile.State(drivetrain.calculateDesiredAngle(),0);
        m_goal = () ->  setpoint;
    }

    public boolean isFinished() {
        return getController().atSetpoint() || limelights.getShooterLimelight().hasValidTarget();
    }

}
