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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Setup components
    SetupMotors();
    SetupEncoders();
    SetupSwitches();  
  }

  // Setup the motors
  private void SetupMotors() {
    // Pivot Motor Config
    m_PivotMotor = new SparkFlex(Constants.ArmSubsystemConstants.ARM_CAN_ID, MotorType.kBrushless);
    m_PivotConfig = new SparkFlexConfig();
    m_PivotConfig.idleMode(IdleMode.kBrake); // Coast or Brake
    m_PivotConfig.inverted(false); // Inverted or Not
    m_PivotConfig.secondaryCurrentLimit(Constants.ArmSubsystemConstants.PIVOT_MOTOR_CURRENT_LIMIT);
    m_PivotMotor.configure(
    (SparkBaseConfig) m_PivotConfig,
    com.revrobotics.ResetMode.kResetSafeParameters,
    com.revrobotics.PersistMode.kPersistParameters);
  }

  // Setup the encoders
  private void SetupEncoders() {
    m_PivotEncoder = new Encoder(Constants.ArmSubsystemConstants.ARM_ENCODER_DIO_A, Constants.ArmSubsystemConstants.ARM_ENCODER_DIO_B);
  }

  // Setup the limit switches
  private void SetupSwitches() {
    m_EndSwitch = new DigitalInput(Constants.ArmSubsystemConstants.ARM_ENDSWITCH_DIO_ID);
    m_StartSwitch = new DigitalInput(Constants.ArmSubsystemConstants.ARM_STARTSWITCH_DIO_ID);
  }

  // Returns true if the arm is at the end position
  public boolean isArmAtEnd() {
    return m_EndSwitch.get();
  }

  // Returns true if the arm is at the start position
  public boolean isArmAtStart() {
    return m_StartSwitch.get();
  }

  // Returns the current pivot tick/position from the encoder
  public int getPivotAngle() {
    return m_PivotEncoder.get();
  }

  // Pivots the arm at the given speed
  public void pivot(double speed) {
    double governedSpeed = Constants.ArmSubsystemConstants.GetSafePivotSpeed(speed);
    m_PivotMotor.set(governedSpeed);
  }

  // Returns a command that will pivot the arm at the given speed while scheduled
  public Command pivot(DoubleSupplier speed) {
    return new RunCommand(
      () -> this.pivot(speed.getAsDouble()),
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
  }
}
