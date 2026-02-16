package frc.lib.util.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TuningManager {
	public static TuningMode tuningMode = TuningMode.Shooter;

	public static SendableChooser<TuningMode> tuningModeChooser = new SendableChooser<TuningMode>();

	public static void init(Swerve swerve, Shooter shooter, Intake intake, Climber climber, Indexer indexer) {
		tuningModeChooser.setDefaultOption("Shooter", TuningMode.Shooter);
		tuningModeChooser.addOption("Swerve", TuningMode.Swerve);
		tuningModeChooser.addOption("Regression", TuningMode.Regression);
		tuningModeChooser.addOption("Intake", TuningMode.Intake);
		tuningModeChooser.addOption("Climber", TuningMode.Climber);
		tuningModeChooser.addOption("Indexer", TuningMode.Indexer);
		SmartDashboard.putData("Tuning Mode", tuningModeChooser);

		if (swerve != null)
			SwerveTuning.init(swerve);
		if (shooter != null)
			ShooterTuning.init(shooter);
		if (shooter != null && swerve != null)
			RegressionTuning.init(swerve, shooter);
		if (intake != null)
			IntakeTuning.init(intake);
		if (climber != null)
			ClimberTuning.init(climber);
		if (indexer != null)
			IndexerTuning.init(indexer);
	}

	public static void changeMode() {
		tuningMode = tuningModeChooser.getSelected();
	}
}
