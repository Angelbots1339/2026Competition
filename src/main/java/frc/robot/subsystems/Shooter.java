// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
	public TalonFX leader = new TalonFX(ShooterConstants.LeaderPort);
	private TalonFX follower = new TalonFX(ShooterConstants.FollowerPort);

	public TalonFX spinner = new TalonFX(ShooterConstants.SpinnerPort);

	private TalonFX indexMotor = new TalonFX(ShooterConstants.IndexPort);

	private double targetShooterRPS = 0.0;
	private double targetSpinnerRPS = 0.0;

	/** Creates a new Shooter. */
	public Shooter() {
		leader.getConfigurator().apply(ShooterConstants.config);
		follower.getConfigurator().apply(ShooterConstants.config);
		spinner.getConfigurator().apply(ShooterConstants.spinnerConfig);
		indexMotor.getConfigurator().apply(ShooterConstants.indexConfig);

		follower.setControl(new Follower(ShooterConstants.LeaderPort, MotorAlignmentValue.Opposed));

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

	public void runIndex(double volts) {
		indexMotor.setVoltage(volts);
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
		setVoltage(Volts.of(0));
	}

	public void disableIndex() {
		runIndex(0);
	}

	public void disable() {
		disableShooter();
		disableIndex();
	}

	@Override
	public void periodic() {
	}
}
