package frc.team3128.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.FieldConstants.*;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;

import static frc.team3128.Constants.VisionConstants.*;

public class LimelightSubsystem extends SubsystemBase{

    public static LimelightSubsystem instance;
    private Limelight m_shooterLimelight;
    private Limelight m_ballLimelight;

    public LimelightSubsystem() {
        m_shooterLimelight = new Limelight("limelight-cog", TOP_CAMERA_ANGLE, TOP_CAMERA_HEIGHT, TOP_FRONT_DIST); 
        m_ballLimelight = new Limelight("limelight-sog", BALL_LL_ANGLE, BALL_LL_HEIGHT, BALL_LL_FRONT_DIST);
    }

    public static synchronized LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    /**
     * Wrapper function to uniformly calculate distance to a target using a limelight
     * @param limelight - name of limelight, "shooter" or "ball"
     */
    public double calculateDistance(String limelight) {
        if (limelight.equalsIgnoreCase("shooter")) {
            // for now, 5 is offset from front of hub to retroreflective tape - will fix later
            return m_shooterLimelight.calculateDistToTopTarget(TARGET_HEIGHT) + 5;
        }
        else if (limelight.equalsIgnoreCase("ball")) {
            // target height is for center of ball
            return m_ballLimelight.calculateDistToGroundTarget(BALL_TARGET_HEIGHT / 2);
        }
        return -1;
    }

    public double calculateTx(String limelight){
        if(limelight.equalsIgnoreCase("shooter")){
            return m_shooterLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 2);
        }
        else if(limelight.equalsIgnoreCase("ball")){
            return m_ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET,2);
        }
        return -1;
    }

    public Pose2d visionEstimatedPose(double gyroAngle) {
        /*DO NOT USE UNLESS LIMELIGHT HAS A VALID TARGET */
        double distToHubCenter = Units.inchesToMeters(calculateDistance("shooter")) + HUB_RADIUS;
        Rotation2d thetaHub = Rotation2d.fromDegrees(gyroAngle - calculateTx("shooter"));
        Translation2d fieldPos = new Translation2d(-distToHubCenter * Math.cos(thetaHub.getRadians()), -distToHubCenter * Math.sin(thetaHub.getRadians()))
                                    .plus(HUB_POSITION.getTranslation());
        return new Pose2d(fieldPos, Rotation2d.fromDegrees(gyroAngle));
    }

    public Limelight getShooterLimelight() {
        return m_shooterLimelight;
    }

    public Limelight getBallLimelight() {
        return m_ballLimelight;
    }

    public void turnShooterLEDOff() {
        m_shooterLimelight.setLEDMode(LEDMode.OFF);
    }

    public void turnShooterLEDOn() {
        m_shooterLimelight.setLEDMode(LEDMode.ON);
    }

}
