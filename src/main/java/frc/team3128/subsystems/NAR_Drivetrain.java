package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.DriveConstants.*;

import java.lang.reflect.Array;

import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.utility.NAR_Shuffleboard;

/**
 * Class for the Drivetrain Subsystem 
 */

public class NAR_Drivetrain extends SubsystemBase {

    // Initialize the generic motors

    private NAR_TalonFX leftLeader;
    private NAR_TalonFX rightLeader;
    private NAR_TalonFX leftFollower;
    private NAR_TalonFX rightFollower;

    private static NAR_Drivetrain instance;

    private DifferentialDrive robotDrive;
    private DifferentialDrivetrainSim robotDriveSim;
    private DifferentialDriveOdometry odometry;

    private double[] xyz_dps = new double[3];

    private static WPI_Pigeon2 gyro = new WPI_Pigeon2(0);

    private static Field2d field;
    
    public NAR_Drivetrain(){
        
        configMotors();

        robotDrive = new DifferentialDrive(
            new MotorControllerGroup(leftLeader, leftFollower),
            new MotorControllerGroup(rightLeader, rightFollower));

        if(RobotBase.isSimulation()){
            robotDriveSim = new DifferentialDrivetrainSim(
                DRIVE_CHAR,
                GEARBOX,
                DRIVE_GEARING,
                TRACK_WIDTH_METERS,
                WHEEL_RADIUS_METERS, 
                null/*VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)*/);
        }

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        field = new Field2d();

        resetEncoders();
    }

    public static synchronized NAR_Drivetrain getInstance() {
        if (instance == null) {
            instance = new NAR_Drivetrain();
        }
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    public void configMotors() {
        leftLeader = new NAR_TalonFX(DRIVE_MOTOR_LEFT_LEADER_ID);
        rightLeader = new NAR_TalonFX(DRIVE_MOTOR_RIGHT_LEADER_ID);
        leftFollower = new NAR_TalonFX(DRIVE_MOTOR_LEFT_FOLLOWER_ID);
        rightFollower = new NAR_TalonFX(DRIVE_MOTOR_RIGHT_FOLLOWER_ID);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        
        leftLeader.setInverted(true);
        leftFollower.setInverted(true);
        rightLeader.setInverted(false);
        rightFollower.setInverted(false);

        leftLeader.setSafetyEnabled(false);
        leftFollower.setSafetyEnabled(false);
        rightLeader.setSafetyEnabled(false);
        rightFollower.setSafetyEnabled(false);

        leftLeader.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        rightLeader.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);

        // set CAN status frame periods
        leftFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        leftFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        leftFollower.setControlFramePeriod(ControlFrame.Control_3_General, 45);

        rightFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        rightFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        rightFollower.setControlFramePeriod(ControlFrame.Control_3_General, 45);

        leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 15);
        leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15);
        leftLeader.setControlFramePeriod(ControlFrame.Control_3_General, 20);
        
        rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 15);
        rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15);
        rightLeader.setControlFramePeriod(ControlFrame.Control_3_General, 20);
    }

    public void initShuffleboard(){
        // General Tab
        NAR_Shuffleboard.addData("General","Gyro",this::getHeading).withPosition(0, 2);
        NAR_Shuffleboard.addData("General","Pitch",this::getPitch).withPosition(1, 2);
        // Drivetrain Tab
        NAR_Shuffleboard.addComplex("Field","field",field).withWidget("Field").withPosition(0,0).withSize(13,7);
        NAR_Shuffleboard.addData("Drivetrain","Left Encoder (m)",this::getLeftEncoderDistance).withPosition(1, 1);
        NAR_Shuffleboard.addData("Drivetrain","Right Encoder (m)",this::getRightEncoderDistance).withPosition(0, 1);
        NAR_Shuffleboard.addData("Drivetrain","Left Encoder Speed(m|s)",this::getLeftEncoderSpeed).withSize(2, 1).withPosition(0,3);
        NAR_Shuffleboard.addData("Drivetrain","Right Encoder (m|s)",this::getRightEncoderSpeed).withSize(2, 1).withPosition(0, 2);
        NAR_Shuffleboard.addData("Drivetrain","Pose",() -> (getPose().toString())).withSize(4, 1).withPosition(2, 0);
        NAR_Shuffleboard.addData("Drivetrain","Gyro",this::getHeading).withPosition(3, 1);
        NAR_Shuffleboard.addData("Drivetrain","Pitch",this::getPitch).withPosition(4, 1);
        NAR_Shuffleboard.addData("Drivetrain","PitchRate",this::getPitchRate).withPosition(5, 1);
        NAR_Shuffleboard.addComplex("Drivetrain","Drivetrain", this).withPosition(0,0);
        // ShuffleboardTab tab = Shuffleboard.getTab("Test");
        // tab.add("Field",field).withWidget("Field");
        if(RobotBase.isSimulation()) {
            NAR_Shuffleboard.addData("Drivetrain","LeftSimSpeed", leftLeader::getSelectedSensorVelocity).withPosition(1,6);
            NAR_Shuffleboard.addData("Drivetrain","RightSimSpeed", rightLeader::getSelectedSensorVelocity).withPosition(0,6);
        }
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
        field.setRobotPose(getPose());

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void simulationPeriodic() {

        // Set motor voltage inputs
        robotDriveSim.setInputs(
            leftLeader.getMotorOutputVoltage(),
            rightLeader.getMotorOutputVoltage()
        );

        // Update sim environment
        robotDriveSim.update(0.02);

        // Store simulated motor states
        leftLeader.setSimPosition(robotDriveSim.getLeftPositionMeters() / DRIVE_NU_TO_METER);
        leftLeader.setSimVelocity(robotDriveSim.getLeftVelocityMetersPerSecond() / DRIVE_NU_TO_METER);
        rightLeader.setSimPosition(robotDriveSim.getRightPositionMeters() / DRIVE_NU_TO_METER);
        rightLeader.setSimVelocity(robotDriveSim.getRightVelocityMetersPerSecond() / DRIVE_NU_TO_METER);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(robotDriveSim.getHeading().getDegrees()); // @Nathan: I tested this out, this seems to work. This preserves parity w/ the real robot in angle, odometry
        SmartDashboard.putNumber("Sim Gyro", angle.get());
    }
        
    public double getHeading() {
        //gyro.getYaw uses CW as positive
        //WPI_Pigeon2 negates this
        return -gyro.getAngle(); 
    }

    public double getPitch() {
        return gyro.getPitch();
    }
   
    public double getPitchRate() {
        gyro.getRawGyro(xyz_dps);
        return xyz_dps[0];
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return the left encoder position in meters
     */
    public double getLeftEncoderDistance() {
        return leftLeader.getSelectedSensorPosition() * DRIVE_NU_TO_METER;
    }

    /**
     * @return the right encoder position in meters
     */
    public double getRightEncoderDistance() {
        return rightLeader.getSelectedSensorPosition() * DRIVE_NU_TO_METER;
    }

    /**
     * @return the left encoder velocity in meters per second
     */
    public double getLeftEncoderSpeed() {
        return leftLeader.getSelectedSensorVelocity() * DRIVE_NU_TO_METER;
    }

    /**
     * @return the right encoder velocity in meters per second
     */
    public double getRightEncoderSpeed() {
        return rightLeader.getSelectedSensorVelocity() * DRIVE_NU_TO_METER;
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
    }

    public void arcadeDrive(double x, double y) {
        robotDrive.arcadeDrive(x, y, false);
    }

    public void stop() {
        robotDrive.stopMotor();
    }

    /**
     * @param leftSpeed the left speed on [-1.0, 1.0]
     * @param rightSpeed the right speed on [-1.0, 1.0]
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed, false);
        robotDrive.feed();
    }

    /**
     * @param leftVolts Left-side voltage on [-12.0, 12.0]
     * @param rightVolts Right-side voltage on [-12.0, 12.0]
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        tankDrive(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage());
        robotDrive.feed();
    }

    /**
     * @param leftVel Left side speed in native encoder units per 100ms
     * @param rightVel right side speed in native encoder units per 100ms
     */
    public void setVelocity(double leftVel, double rightVel) {
        // leftLeader.set(ControlMode.Velocity, leftVel);
        // rightLeader.set(ControlMode.Velocity, rightVel);
        // robotDrive.feed();

        tankDrive(leftVel / MAX_DRIVE_VEL_NUp100MS, rightVel / MAX_DRIVE_VEL_NUp100MS);
    }

    /**
     * @param leftVelMpS left side speed in meters per second
     * @param rightVelMps right side speed in meters per second
     */
    public void setVelocityMpS(double leftVelMpS, double rightVelMps) {
        setVelocity(leftVelMpS / DRIVE_NUp100MS_TO_MPS, rightVelMps / DRIVE_NUp100MS_TO_MPS);
    }

    public void resetEncoders() {
        leftLeader.setEncoderPosition(0);
        rightLeader.setEncoderPosition(0);
    }

    public void resetPose(Pose2d poseMeters) {
        resetEncoders();
        odometry.resetPosition(poseMeters, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Reset pose to (x = 0, y = 0, theta = 0)
     */
    public void resetPose() {
        resetGyro();
        resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading())));
    }

    public void resetGyro() {
        gyro.reset();
    }

}

