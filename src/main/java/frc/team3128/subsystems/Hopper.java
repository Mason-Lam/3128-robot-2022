package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.hardware.NAR_CANSparkMax;
import frc.team3128.hardware.NAR_TalonSRX;

public class Hopper implements Subsystem {

    private NAR_TalonSRX m_hopper_1;
    private NAR_CANSparkMax m_hopper_2;

    private DigitalInput BOTTOM_SENSOR, TOP_SENSOR;

    private TalonSRXSimCollection m_hopper_sim_1;


    public Hopper() {
        configMotors();
        configSensors();
    }

    private void configMotors() {
        m_hopper_1 = new NAR_TalonSRX(Constants.HopperConstants.HOPPER_MOTOR_1_ID);
        m_hopper_2 = new NAR_CANSparkMax(Constants.HopperConstants.HOPPER_MOTOR_2_ID, MotorType.kBrushless);
    }

    private void configSensors() {
        BOTTOM_SENSOR = new DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID);
        TOP_SENSOR = new DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
    }

    public boolean getBottom() {
        return !BOTTOM_SENSOR.get();
    }

    public boolean getTop() {
        return !TOP_SENSOR.get();
    }

    // these should be changed to commands 
    public void runHopper(double multiplier) {
        m_hopper_1.set(ControlMode.PercentOutput, Constants.HopperConstants.HOPPER_MOTOR_1_POWER*multiplier);
        m_hopper_2.set(Constants.HopperConstants.HOPPER_MOTOR_2_POWER);
    }

    public void stopHopper() {
        m_hopper_1.set(ControlMode.PercentOutput, 0);
        m_hopper_2.set(0);
    }

    @Override
    public void simulationPeriodic() {

    }

}
