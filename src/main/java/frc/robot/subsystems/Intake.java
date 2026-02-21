// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.tuning.IntakeTuning;
import frc.robot.Constants.IntakeConstants;

@Logged
public class Intake extends SubsystemBase {
	private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorId, "*");
	private TalonFX deployMotor = new TalonFX(IntakeConstants.deployMotorId);
	private Angle targetAngle = Degrees.zero();

	public Intake() {
		deployMotor.getConfigurator().apply(IntakeConstants.deployConfigs);
		intakeMotor.getConfigurator().apply(IntakeConstants.intakeConfigs);

		deployMotor.setPosition(IntakeConstants.MaxAngle);
	}

	public Command deploy() {
		return run(() -> setIntakeAngle(IntakeConstants.DeployedAngle));
	}

	public Command retract() {
		return run(() -> setIntakeAngle(IntakeConstants.RetractedAngle));
	}

	public Command runIntake() {
		return run(() -> {
			setIntakeAngle(IntakeConstants.DeployedAngle);
			setIntakeVoltage(IntakeConstants.IntakeVoltage);
		});
	}

	public Command stopIntake() {
		return run(() -> setIntakeVoltage(0));
	}

	public void setIntakeAngle(Angle angle) {
		targetAngle = angle;
		deployMotor.setControl(new MotionMagicVoltage(angle));
	}

	public Angle getIntakeAngle() {
		return deployMotor.getPosition().getValue();
	}

	public double getIntakeAngleDegrees() {
		return getIntakeAngle().in(Degrees);
	}

	public boolean isAtSetpoint() {
		return getIntakeAngle().isNear(targetAngle, IntakeConstants.IntakeAngleTolerence);
	}

	public void setIntakeVelocity(double Velocity) {
		intakeMotor.setControl(new VelocityVoltage(Velocity));
	}

	public void setIntakeVoltage(double volts) {
		intakeMotor.setControl(new VoltageOut(volts));
	}

	public void disable() {
		setIntakeVoltage(0);
		deployMotor.setControl(new NeutralOut());
	}

	public void resetPosition(Angle angle) {
		deployMotor.setPosition(angle);
	}

	public void logTuning() {
		IntakeTuning.logPID("Intake/Deploy PID", deployMotor, IntakeConstants.deployConfigs);
	}

	@Override
	public void periodic() {
	}

}
