package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import static frc.team3128.Constants.ShooterConstants.*;
import frc.team3128.ConstantsInt;
import static frc.team3128.Constants.ConversionConstants.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;

public class Shooter extends PIDSubsystem {

    private static Shooter instance;

    //Motors
    private NAR_TalonFX m_leftShooter;
    private NAR_TalonFX m_rightShooter;

    private double currTime = 0, prevTime = 0;
    private int plateauCount = 0;
    private double thresholdPercent = RPM_THRESHOLD_PERCENT;

    private FlywheelSim m_shooterSim;

    public Shooter() {
        super(new PIDController(kP, kI, kD), PLATEAU_COUNT);
    
        configMotors();

        //Robot is a simulation
        if(RobotBase.isSimulation()){
            m_shooterSim = new FlywheelSim(
                SHOOTER_CHAR, SHOOTER_GEARBOX, SHOOTER_GEARING 
            );
        }
    }

    public static synchronized Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private void configMotors() {
        m_leftShooter = new NAR_TalonFX(LEFT_SHOOTER_ID);
        m_rightShooter = new NAR_TalonFX(RIGHT_SHOOTER_ID);

        m_leftShooter.setInverted(false);
        m_rightShooter.setInverted(true);

        m_rightShooter.follow(m_leftShooter);

        // set CAN status frame periods
        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        m_rightShooter.setControlFramePeriod(ControlFrame.Control_3_General, 45);
        
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 15);
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15);
        m_leftShooter.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        m_leftShooter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 30, 0.1));
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Shooter Setpoint", getSetpoint());
        SmartDashboard.putNumber("Shooter RPM", getMeasurement());
        SmartDashboard.putBoolean("Shooter isReady", isReady());
    }

    /**
     * Begins the PID loop to achieve the desired RPM to shoot
     */
    public void beginShoot(double rpm) {
        thresholdPercent = RPM_THRESHOLD_PERCENT;
        prevTime = RobotController.getFPGATime() / 1e6;
        plateauCount = 0;

        // rpm = ConstantsInt.ShooterConstants.SET_RPM; // uncomment for interpolation
        setSetpoint(rpm);
        getController().setTolerance(RPM_THRESHOLD_PERCENT * rpm);
    }

    public void stopShoot() {
        plateauCount = 0;
        setSetpoint(0);
    }

    @Override
    public double getMeasurement() {
        return m_leftShooter.getSelectedSensorVelocity() * FALCON_NUpS_TO_RPM;
    }

    /**
     * Use the raw voltage output from the PID loop, add a feed forward component, and convert it to a percentage of total
     * possible voltage to apply to the motors.
     * 
     * @param output Output from the PID Loop (RPM)
     * @param setpoint The desired setpoint RPM for the PID Loop (RPM)
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = kF * setpoint;
        double voltageOutput = output + ff;

        // Increase the tolerance as time goes on (unsure if this is needed, but we want to shoot better late than never)
        currTime = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < RPM_THRESHOLD_PERCENT_MAX) {
            thresholdPercent += (currTime - prevTime) * (RPM_THRESHOLD_PERCENT_MAX - RPM_THRESHOLD_PERCENT) / TIME_TO_MAX_THRESHOLD;
            getController().setTolerance(thresholdPercent * setpoint);
        }

        if (getController().atSetpoint() && (setpoint != 0)) {
            plateauCount ++;
        } else {
            plateauCount = 0;
        }

        if (setpoint == 0)
            voltageOutput = 0;
            
        m_leftShooter.set(ControlMode.PercentOutput, MathUtil.clamp(voltageOutput / 12.0, -1, 1));

        prevTime = currTime;
    }

    public boolean isReady() {
        return (plateauCount >= PLATEAU_COUNT) && (getSetpoint() != 0);
    }

    @Override
    public void simulationPeriodic() {
        m_shooterSim.setInput(
            m_leftShooter.getMotorOutputVoltage()
        );  
        m_shooterSim.update(0.02);    
        
        m_leftShooter.setSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * SHOOTER_RADIUS_METERS);
        //m_rightShooter.setQuadSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * SHOOTER_RADIUS_METERS);
    
        // SmartDashboard.putNumber("test", m_leftShooter.getMotorOutputVoltage()); 
        // SmartDashboard.putString("pogger", String.valueOf(m_shooterSim.getAngularVelocityRadPerSec()));
        SmartDashboard.putNumber("shooter RPM", m_shooterSim.getAngularVelocityRadPerSec() * 60 / (2*Math.PI));
        
    }

    public double calculateMotorVelocityFromDist(double dist) {
        return shooterSpeedsMap.getInterpolated(new InterpolatingDouble(dist)).value;
    }
}

