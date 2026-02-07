// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpinSubsystem extends SubsystemBase {

  private Servo spinner;
  
  /** Creates a new SpinSubsystem. */
  public SpinSubsystem() {
    // Setup components
    SetupServo();
  }

  // Initializes the servo and sets it to the min position
  private void SetupServo() {
    spinner = new Servo(Constants.SpinSubsystemConstants.SPIN_SERVO_PWM_ID);
    setPosition(Constants.SpinSubsystemConstants.SPIN_SERVO_MIN_POSITION);
  }

  // Gets the current angle of the servo
  public double getAngle() {
    return spinner.getAngle();
  }

  // Gets the current position of the servo  
  public double getPosition() {
    return spinner.get(); 
  }

  // Sets the angle of the servo, where 0.0 is fully stowed and 180.0 is fully deployed
  public void setAngle(double angle) {
    if (angle < Constants.SpinSubsystemConstants.SPIN_SERVO_MIN_ANGLE) {
      angle = Constants.SpinSubsystemConstants.SPIN_SERVO_MIN_ANGLE;
    } else if (angle > Constants.SpinSubsystemConstants.SPIN_SERVO_MAX_ANGLE) {
      angle = Constants.SpinSubsystemConstants.SPIN_SERVO_MAX_ANGLE;
    }
    spinner.setAngle(angle);
  } 

  // Sets the position of the servo, where 0.0 is fully stowed and 1.0 is fully deployed
  public void setPosition(double position) {
    if (position < Constants.SpinSubsystemConstants.SPIN_SERVO_MIN_POSITION) {
      position = Constants.SpinSubsystemConstants.SPIN_SERVO_MIN_POSITION;
    } else if (position > Constants.SpinSubsystemConstants.SPIN_SERVO_MAX_POSITION) {
      position = Constants.SpinSubsystemConstants.SPIN_SERVO_MAX_POSITION;
    }
    spinner.set(position);
  }

  // Returns a command that will spin to position
  public Command commandToPosition(double position) {
    return new RunCommand(
      () -> this.setPosition(position),
      this);
  }

  // Returns a command that will spin to angle
  public Command commandToAngle(double angle) {
    return new RunCommand(
      () -> this.setAngle(angle),
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Spinner Position", getPosition()); 
    SmartDashboard.putNumber("Spinner Angle", getAngle());
  }
}
