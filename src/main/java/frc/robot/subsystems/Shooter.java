// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

	private TalonFX spinner = new TalonFX(ShooterConstants.SpinnerPort);

	private double targetShooterRPS = 0.0;
	private double targetSpinnerRPS = 0.0;

	/** Creates a new Shooter. */
	public Shooter() {
		leader.getConfigurator().apply(ShooterConstants.config);
		follower.getConfigurator().apply(ShooterConstants.config);
		spinner.getConfigurator().apply(ShooterConstants.spinnerConfig);

		follower.setControl(new Follower(ShooterConstants.LeaderPort, MotorAlignmentValue.Aligned));

		leader.getVelocity().setUpdateFrequency(Hertz.of(100));
		spinner.getVelocity().setUpdateFrequency(Hertz.of(100));
	}

	public void setVoltage(Voltage volts) {
		leader.setControl(new VoltageOut(volts));
		spinner.setControl(new VoltageOut(volts));
	}

	public void setVelocity(double rps) {
		targetShooterRPS = rps;
		leader.setControl(new VelocityVoltage(rps));
	}

	public void setVelocityFOC(double shooterRPS, double spinnerRPS) {
		targetShooterRPS = shooterRPS;
		targetSpinnerRPS = spinnerRPS;
		leader.setControl(ShooterConstants.velocityTorqueControl.withVelocity(targetShooterRPS));
		spinner.setControl(ShooterConstants.velocityTorqueControl.withVelocity(targetSpinnerRPS));
	}

	public void setVelocityFOC(double rps) {
		setVelocityFOC(rps, rps);
	}

	public double getShooterRPS() {
		return leader.getVelocity().getValue().in(RotationsPerSecond);
	}

	public double getSpinnerRPS() {
		return spinner.getVelocity().getValue().in(RotationsPerSecond);
	}

	public void logTuning() {
		SmartDashboard.putNumber(TuningConstants.Shooter.voltageNTName, 0);
		SmartDashboard.putNumber(TuningConstants.Shooter.velocityNTName, 0);

		SmartDashboard.putNumber(TuningConstants.Shooter.PNTName, ShooterConstants.config.Slot1.kP);
		SmartDashboard.putNumber(TuningConstants.Shooter.INTName, ShooterConstants.config.Slot1.kI);
		SmartDashboard.putNumber(TuningConstants.Shooter.DNTName, ShooterConstants.config.Slot1.kD);
		SmartDashboard.putNumber(TuningConstants.Shooter.SNTName, ShooterConstants.config.Slot1.kS);
		SmartDashboard.putNumber(TuningConstants.Shooter.VNTName, ShooterConstants.config.Slot1.kV);

	}

	public Slot0Configs getPID() {
		double p = SmartDashboard.getNumber(TuningConstants.Shooter.PNTName, 0);
		double i = SmartDashboard.getNumber(TuningConstants.Shooter.INTName, 0);
		double d = SmartDashboard.getNumber(TuningConstants.Shooter.DNTName, 0);
		double s = SmartDashboard.getNumber(TuningConstants.Shooter.SNTName, 0);
		double v = SmartDashboard.getNumber(TuningConstants.Shooter.VNTName, 0);

		return new Slot0Configs()
				.withKP(p)
				.withKI(i)
				.withKD(d)
				.withKV(v)
				.withKS(s);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
