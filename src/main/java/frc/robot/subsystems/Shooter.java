// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase {
	private TalonFX leader = new TalonFX(ShooterConstants.LeaderPort);
	private TalonFX follower = new TalonFX(ShooterConstants.FollowerPort);
	private TalonFX follower2 = new TalonFX(ShooterConstants.Follower2Port);

	/** Creates a new Shooter. */
	public Shooter() {
		leader.getConfigurator().apply(ShooterConstants.config);
		follower.getConfigurator().apply(ShooterConstants.config);
		follower.setControl(new Follower(ShooterConstants.LeaderPort, MotorAlignmentValue.Opposed));
		follower2.setControl(new Follower(ShooterConstants.LeaderPort, MotorAlignmentValue.Aligned));
	}

	public void setVoltage(Voltage volts) {
		leader.setControl(new VoltageOut(volts));
	}

	public void setVoltage() {
		setVoltage(Volts.of(SmartDashboard.getNumber(TuningConstants.Shooter.voltageNTName, 0)));
	}

	public AngularVelocity getVelocity() {
		return leader.getVelocity().getValue();
	}

	public void logTuning() {
		SmartDashboard.putNumber("voltage", 0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
