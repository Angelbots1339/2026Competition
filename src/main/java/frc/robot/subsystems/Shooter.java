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
		// for all of our motors, we set the configuration for each motor for
		// current limits, polarity, encoder settings, closed loop gains, etc
		leader.getConfigurator().apply(ShooterConstants.ShooterConfig);
		follower.getConfigurator().apply(ShooterConstants.ShooterConfig);
		follower2.getConfigurator().apply(ShooterConstants.ShooterConfig);
		spinner.getConfigurator().apply(ShooterConstants.SpinnerConfig);
		kicker.getConfigurator().apply(ShooterConstants.KickerConfig);

		// the robot has a 3 motor setup for the main flywheel, and they all
		// drive the same axel.  we can use followers so that we only control
		// the output of one motor, which automatically forwards the commanded
		// output, with some changes in polarity.
		follower.setControl(new Follower(ShooterConstants.Shooter1Port, MotorAlignmentValue.Opposed));
		follower2.setControl(new Follower(ShooterConstants.Shooter1Port, MotorAlignmentValue.Opposed));

		// we set the update frequency of the getVelocity call so that we can get
		// / log the data at a faster rate
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
		// the reason for the custum velocitytorquecontrol function is to set
		// specific configurations, namely the update frequency
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

	// NEVER USE the getClosedLoopError provided by ctre as this value is delayed
	// and can be temporarily at the wrong value even after commanding it to a
	// different setpoint
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

	// this is so that we can access the motor pid configuration in our test
	// modes to tune PID
	public void logPID() {
		TuningManager.createPID("Shooter/leader", leader, ShooterConstants.ShooterConfig);
		TuningManager.createPID("Shooter/spinner", spinner, ShooterConstants.SpinnerConfig);
		TuningManager.createPID("Shooter/kicker", kicker, ShooterConstants.KickerConfig);
	}

	@Override
	public void periodic() {
	}
}
