package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
	private TalonFX indexerMotor = new TalonFX(IndexerConstants.IndexerMotorPort);

	public Indexer() {
		indexerMotor.getConfigurator().apply(IndexerConstants.IndexerMotorConfig);
	}

	public void runVoltage(double volts) {
		indexerMotor.setControl(new VoltageOut(volts));
	}

	public void runVelocity(double velocity) {
		indexerMotor.setControl(new VelocityTorqueCurrentFOC(velocity));
	}

	public void disable() {
		indexerMotor.setControl(new NeutralOut());
	}

	@Override

	public void periodic() {
	}
}
