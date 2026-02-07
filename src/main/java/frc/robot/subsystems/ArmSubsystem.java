// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmSubsystemConstants.DIRECTION;

public class ArmSubsystem extends SubsystemBase {

  // Subsystem Components
  private DigitalInput m_EndSwitch;
  private DigitalInput m_StartSwitch;
  private Encoder m_PivotEncoder;
  private SparkFlex m_PivotMotor;
  private SparkFlexConfig m_PivotConfig;
  private PIDController m_PivotPID;
  private double kP, kI, kD;
  private Pigeon2 m_IMU;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Setup components
    SetupMotors();
    SetupEncoders();
    SetupSwitches(); 
    SetupPIDController(); 
    SetupIMU();
  }

  // Setup the motors
  private void SetupMotors() {
    // Pivot Motor Config
    m_PivotMotor = new SparkFlex(Constants.ArmSubsystemConstants.ARM_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
    m_PivotConfig = new SparkFlexConfig();
    m_PivotConfig.idleMode(IdleMode.kBrake); // Coast or Brake
    m_PivotConfig.inverted(true); // Inverted or Not
    m_PivotConfig.secondaryCurrentLimit(Constants.ArmSubsystemConstants.PIVOT_MOTOR_CURRENT_LIMIT);
    m_PivotMotor.configure(
    (SparkBaseConfig) m_PivotConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  // Setup the encoders
  private void SetupEncoders() {
    m_PivotEncoder = new Encoder(Constants.ArmSubsystemConstants.ARM_ENCODER_DIO_A, Constants.ArmSubsystemConstants.ARM_ENCODER_DIO_B);
    m_PivotEncoder.setReverseDirection(true);
    m_PivotEncoder.reset();
  }

  // Setup PID controller
  private void SetupPIDController() {
    // Sets PID coefficients for the Pivot neo
    kP = 0.001;
    kI = 0;
    kD = 0;
    m_PivotPID = new PIDController(kP, kI, kD);
    m_PivotPID.setTolerance(Constants.ArmSubsystemConstants.ARM_PIVOT_PID_TOLERANCE);
  }

  // Setup the limit switches
  private void SetupSwitches() {
    m_EndSwitch = new DigitalInput(Constants.ArmSubsystemConstants.ARM_ENDSWITCH_DIO_ID);
    m_StartSwitch = new DigitalInput(Constants.ArmSubsystemConstants.ARM_STARTSWITCH_DIO_ID);
  }

  // Setup the IMU
  private void SetupIMU() {
    m_IMU = new Pigeon2(Constants.ArmSubsystemConstants.ARM_IMU_CAN_ID);
    m_IMU.clearStickyFaults();
    m_IMU.reset();
  }

  // Returns true if the arm is at the end position
  public boolean isArmAtEnd() {
    return !m_EndSwitch.get();
  }

  // Returns true if the arm is at the start position
  public boolean isArmAtStart() {
    return m_StartSwitch.get();
  }

  // Returns the current pivot tick/position from the encoder
  // Encoder: 2048 ticks equals 1 revolution or 360 degrees
  public double getPivotAngle() {
    return m_PivotEncoder.get() * 0.176;
  }

  // Returns the current raw tick count from the encoder.  These are ticks from the last ZERO position
  public double getPivotTicks() {
    return m_PivotEncoder.get();
  }

  // When called it resets encoder such that the current arm position becomes ZERO position
  public void resetPivotEncoder() {
    m_PivotEncoder.reset();
  }

  // Pivots the arm at the given speed
  public void pivot(double speed) {
    double governedSpeed = Constants.ArmSubsystemConstants.GetSafePivotSpeed(speed);
    m_PivotMotor.set(governedSpeed);
  }

  // Pivots based on requested setPoint
  public void movePivotWithPID(double currentDistance, double setPoint) {
    System.out.println("Current Distance: " + currentDistance);
    m_PivotMotor.set(
        MathUtil.clamp(
            m_PivotPID.calculate(currentDistance, setPoint),
            Constants.ArmSubsystemConstants.ARM_PIVOT_PID_MIN_OUTPUT,
            Constants.ArmSubsystemConstants.ARM_PIVOT_PID_MAX_OUTPUT));
  }

  // Pivot PID error
  public double getPivotPIDError() {
    return m_PivotPID.getError();
  }

  // Get PitvotPID at set point property
  public boolean getPivotPIDAtSetpoint() {
    return m_PivotPID.atSetpoint();
  }

  // Get current IMU angle
  public Double getIMUPitch() {
    return m_IMU.getPitch().getValueAsDouble();
  }

  // Get current IMU Yaw
  public double getIMUYaw() {
    return m_IMU.getYaw().getValueAsDouble();
  }

  // Get current IMU Roll
  public double getIMURoll() {
    return m_IMU.getRoll().getValueAsDouble();
  }

  public double getAccelerationX() {
    return m_IMU.getAccelerationX().getValueAsDouble();
  }

  public double getGravityX() {
    return m_IMU.getGravityVectorX().getValueAsDouble();  
  } 
  
  // Returns a command that will pivot the arm at the given speed while scheduled
  public Command pivot(DoubleSupplier speed) {
    return new RunCommand(
      () -> this.pivot(speed.getAsDouble()),
      this);
  }

  // Returns a command that will reset the encoder position to zero
  public Command resetEncoder() {
    return new RunCommand(
      () -> this.resetPivotEncoder(),
      this);
  }

  // **WARNING** UNTESTED - need to verify once hardware is available
  public DIRECTION getPivotDirection() {
    if (m_PivotMotor.get() > 0) {
      return DIRECTION.FORWARD;
    } else {
      return DIRECTION.REVERSE;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Arm Pivot Ticks", getPivotTicks());
    SmartDashboard.putBoolean("Arm at Start", isArmAtStart());
    SmartDashboard.putBoolean("Arm at End", isArmAtEnd());
    SmartDashboard.putNumber("Arm Yaw", getIMUYaw());
    SmartDashboard.putNumber("Arm Roll", getIMURoll());
    SmartDashboard.putNumber("Arm Pitch", getIMUPitch());
    SmartDashboard.putNumber("Arm Acceleration X", getAccelerationX());
    SmartDashboard.putNumber("Arm Gravity X", getGravityX()); 
  }
}
