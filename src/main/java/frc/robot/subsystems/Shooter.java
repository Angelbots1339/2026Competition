// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.tuning.TuningManager;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
	private TalonFX leader = new TalonFX(ShooterConstants.Shooter1Port);
	private TalonFX follower = new TalonFX(ShooterConstants.Shooter2Port);
	private TalonFX follower2 = new TalonFX(ShooterConstants.Shooter3Port);

	private TalonFX spinner = new TalonFX(ShooterConstants.SpinnerPort);

	private TalonFX kicker = new TalonFX(ShooterConstants.KickerPort);

	private double targetShooterRPS = 0.0;
	private double targetSpinnerRPS = 0.0;

	/** Creates a new Shooter. */
	public Shooter() {
		leader.getConfigurator().apply(ShooterConstants.ShooterConfig);
		follower.getConfigurator().apply(ShooterConstants.ShooterConfig);
		follower2.getConfigurator().apply(ShooterConstants.ShooterConfig);
		spinner.getConfigurator().apply(ShooterConstants.SpinnerConfig);
		kicker.getConfigurator().apply(ShooterConstants.KickerConfig);

		follower.setControl(new Follower(ShooterConstants.Shooter1Port, MotorAlignmentValue.Opposed));
		follower2.setControl(new Follower(ShooterConstants.Shooter1Port, MotorAlignmentValue.Aligned));

		leader.getVelocity().setUpdateFrequency(Hertz.of(100));
		spinner.getVelocity().setUpdateFrequency(Hertz.of(100));
	}

	public void setVoltage(Voltage volts) {
		leader.setControl(new VoltageOut(volts));
		spinner.setControl(new VoltageOut(volts));
	}

	public void setRPS(double shooterRPS, double spinnerRPS) {
		targetShooterRPS = shooterRPS;
		targetSpinnerRPS = spinnerRPS;
		leader.setControl(ShooterConstants.velocityTorqueControl.withVelocity(targetShooterRPS));
		spinner.setControl(new VelocityTorqueCurrentFOC(targetSpinnerRPS));
	}

	public void setRPS(double rps) {
		setRPS(rps, rps);
	}

	public void setKickerVelocity(double rps) {
		kicker.setControl(new VelocityTorqueCurrentFOC(rps));
	}

	public double getShooterRPS() {
		return leader.getVelocity().getValue().in(RotationsPerSecond);
	}

	public double getSpinnerRPS() {
		return spinner.getVelocity().getValue().in(RotationsPerSecond);
	}

	public double getShooterError() {
		return Math.abs(leader.getVelocity().getValueAsDouble() - targetShooterRPS);
	}

	public boolean atSetpoint() {
		return getShooterError() < ShooterConstants.rpsTolerence;
	}

	public void disableShooter() {
		targetShooterRPS = targetSpinnerRPS = 0;
		setVoltage(Volts.of(0));
	}

	public void disableKicker() {

		kicker.setControl(new NeutralOut());
	}

	public void disable() {
		disableShooter();
		disableKicker();
	}

	public Command unstuck() {
		return run(() -> {
			disableShooter();
			setKickerVelocity(-ShooterConstants.KickerRPS);
		}).withTimeout(2);
	}

	public void logPID() {
		TuningManager.createPID("Shooter/leader", leader, ShooterConstants.ShooterConfig);
		TuningManager.createPID("Shooter/spinner", spinner, ShooterConstants.SpinnerConfig);
		TuningManager.createPID("Shooter/kicker", kicker, ShooterConstants.KickerConfig);
	}

	@Override
	public void periodic() {
	}
}
