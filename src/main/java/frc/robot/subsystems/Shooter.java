// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
	public TalonFX leader = new TalonFX(ShooterConstants.LeaderPort);
	private TalonFX follower = new TalonFX(ShooterConstants.FollowerPort);

	@Logged(importance = Importance.CRITICAL)
	private double targetRPS = 0;

	/** Creates a new Shooter. */
	public Shooter() {
		leader.getConfigurator().apply(ShooterConstants.config);
		follower.getConfigurator().apply(ShooterConstants.config);

		follower.setControl(new Follower(ShooterConstants.LeaderPort, MotorAlignmentValue.Opposed));

		leader.getVelocity().setUpdateFrequency(Hertz.of(100));
	}

	public void setVoltage(Voltage volts) {
		leader.setControl(new VoltageOut(volts));
	}

	public void setVelocityFOC(double rps) {
		targetRPS = rps;
		leader.setControl(ShooterConstants.velocityTorqueControl.withVelocity(targetRPS));
	}

	@Logged(name = "RPS", importance = Importance.CRITICAL)
	public double getVelocity() {
		return leader.getVelocity().getValue().in(RotationsPerSecond);
	}

	@Override
	public void periodic() {
	}
}
