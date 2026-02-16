package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
	private TalonFX indexerMotor = new TalonFX(IndexerConstants.IndexerMotorPort);

	public Indexer() {
		indexerMotor.getConfigurator().apply(IndexerConstants.IndexerMotorConfig);
	}
}
