// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.tuning.IntakeTuning;
import frc.robot.Constants.IntakeConstants;

@Logged()
public class Intake extends SubsystemBase {
	private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorId);
	private TalonFX deployMotor = new TalonFX(IntakeConstants.deployMotorId);

	public Intake() {
		deployMotor.getConfigurator().apply(IntakeConstants.deployConfigs);
		intakeMotor.getConfigurator().apply(IntakeConstants.intakeConfigs);
	}

	public void setIntakeAngle(Angle angle) {
		deployMotor.setControl(new PositionVoltage(angle));
	}

	public double getIntakeAngle() {
		return deployMotor.getPosition().getValue().in(Degrees);
	}

	public void setIntakeVelocity(double Velocity) {
		intakeMotor.setControl(new VelocityVoltage(Velocity));
	}

	public void setIntakeVoltage(double volts) {
		intakeMotor.setControl(new VoltageOut(volts));
	}

	public void disable() {
		setIntakeVoltage(0);
		deployMotor.setControl(new VoltageOut(0));
	}

	public void logTuning() {
		IntakeTuning.logPID("Intake/Deploy PID", deployMotor, IntakeConstants.deployConfigs);
	}

	@Override
	public void periodic() {
	}
}
