// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorId);
  TalonFX deployMotor = new TalonFX(IntakeConstants.deployMotorId);

  public Intake() {
    deployMotor.getConfigurator().apply(IntakeConstants.deployConfigs);
    intakeMotor.getConfigurator().apply(IntakeConstants.intakeConfigs)
  }

  @Override
  public void periodic() {
    
  }
  
  public void setWristAngle(double angle) {
      deployMotor.setControl(new PositionVoltage(angle));
  }

  public double getWristAngle() {
    return deployMotor.getPosition().getValueAsDouble();
  }

  public void setIntakeVelocity(double Velocity) {
    intakeMotor.setControl(new VelocityVoltage(Velocity));
  }
}
