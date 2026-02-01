// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.tuning.ShooterTuning;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningConstants;

@Logged
public class Shooter extends SubsystemBase {
	private TalonFX frontShooter = new TalonFX(ShooterConstants.LeaderPort);
	private TalonFX backShooter = new TalonFX(ShooterConstants.FollowerPort);

	private TalonFX spinner = new TalonFX(ShooterConstants.SpinnerPort);

	@Logged(importance = Importance.CRITICAL)
	private double shooterTargetRPS = 0;

	@Logged(importance = Importance.CRITICAL)
	private double spinnerTargetRPS = 0;

	/** Creates a new Shooter. */
	public Shooter() {
		frontShooter.getConfigurator().apply(ShooterConstants.baseConfig);
		backShooter.getConfigurator().apply(ShooterConstants.baseConfig);
		spinner.getConfigurator().apply(ShooterConstants.spinnerConfig);

		backShooter.setControl(new Follower(ShooterConstants.LeaderPort, MotorAlignmentValue.Aligned));

		frontShooter.getVelocity().setUpdateFrequency(Hertz.of(100));
		spinner.getVelocity().setUpdateFrequency(Hertz.of(100));
	}

	public void setVoltage(Voltage volts) {
		frontShooter.setControl(new VoltageOut(volts));
		spinner.setControl(new VoltageOut(volts));
	}

	public void setRPS(double rps) {
		setRPS(rps, rps);
	}

	public void setRPS(double shooterRPS, double spinnerRPS) {
		shooterTargetRPS = shooterRPS;
		spinnerTargetRPS = spinnerRPS;

		frontShooter.setControl(ShooterConstants.velocityTorqueControl.withVelocity(shooterTargetRPS));
		spinner.setControl(ShooterConstants.velocityTorqueControl.withVelocity(spinnerTargetRPS));
	}

	public void disable() {
		frontShooter.set(0);
		spinner.set(0);
	}

	@Logged(name = "Shooter RPS", importance = Importance.CRITICAL)
	public double getShooterRPS() {
		return frontShooter.getVelocity().getValueAsDouble();
	}

	@Logged(name = "Spinner RPS", importance = Importance.CRITICAL)
	public double getSpinnerRPS() {
		return spinner.getVelocity().getValueAsDouble();
	}

	@Override
	public void periodic() {
	}

	public void logPIDTuning() {
		ShooterTuning.createMotorPID(TuningConstants.ShooterTuningConstants.ShooterVelocityPIDNTName,
				frontShooter,
				ShooterConstants.baseConfig);
		ShooterTuning.createMotorPID(TuningConstants.ShooterTuningConstants.SpinnerVelocityPIDNTName,
				spinner,
				ShooterConstants.spinnerConfig);
	}
}
