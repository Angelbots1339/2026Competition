package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.tuning.TuningManager;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {
	private TalonFX indexerMotor = new TalonFX(IndexerConstants.IndexerMotorPort);

	public Indexer() {
		indexerMotor.getConfigurator().apply(IndexerConstants.IndexerMotorConfig);
	}

	public Command index() {
		return run(() -> {
			runVoltage(IndexerConstants.IndexerVolts);
		});
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

	public void logPID() {
		TuningManager.createPID("Indexer/Indexer Velocity PID", indexerMotor,
				IndexerConstants.IndexerMotorConfig);
	}

	@Override
	public void periodic() {
	}
}
