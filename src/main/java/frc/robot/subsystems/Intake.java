// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public WPI_TalonSRX intake = new WPI_TalonSRX(2);
  public Intake() {
    intake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeForward(){
    intake.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }

  public void intakeBackwards(){
    intake.set(ControlMode.PercentOutput, -1 *Constants.INTAKE_SPEED);
  }

  public void stop(){
    intake.set(ControlMode.PercentOutput, 0);
  }
}
