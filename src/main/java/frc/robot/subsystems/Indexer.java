package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {
	private TalonFX indexerMotor = new TalonFX(IndexerConstants.indexerMotorPort);

	public Indexer() {
		indexerMotor.getConfigurator().apply(IndexerConstants.config);
	}

	public void setVoltage(Voltage volts) {
		indexerMotor.setControl(new VoltageOut(volts));
	}

	public void setTorqueControl(Current amps) {
		indexerMotor.setControl(new TorqueCurrentFOC(amps));
	}

	public void disable() {
		indexerMotor.set(0);
	}

	@Override
	public void periodic() {
	}
}
