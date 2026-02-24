package frc.lib.util.tuning;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TuningManager {
	public static TuningMode tuningMode = TuningMode.Shooter;

	public static SendableChooser<TuningMode> tuningModeChooser = new SendableChooser<TuningMode>();

	public static void init(Swerve swerve, Shooter shooter, Intake intake, Climber climber) {
		tuningModeChooser.setDefaultOption("Shooter", TuningMode.Shooter);
		tuningModeChooser.addOption("Swerve", TuningMode.Swerve);
		tuningModeChooser.addOption("Regression", TuningMode.Regression);
		tuningModeChooser.addOption("Intake", TuningMode.Intake);
		tuningModeChooser.addOption("Climber", TuningMode.Climber);
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
	}

	public static void changeMode() {
		tuningMode = tuningModeChooser.getSelected();
	}

	public static void createPID(String key, TalonFX motor, TalonFXConfiguration config) {
		DogLog.tunable(key + "/kP", config.Slot0.kP,
				newP -> motor.getConfigurator().apply(config.Slot0.withKP(newP)));
		DogLog.tunable(key + "/kI", config.Slot0.kI,
				newI -> motor.getConfigurator().apply(config.Slot0.withKI(newI)));
		DogLog.tunable(key + "/kD", config.Slot0.kD,
				newD -> motor.getConfigurator().apply(config.Slot0.withKD(newD)));
		DogLog.tunable(key + "/kS", config.Slot0.kS,
				newS -> motor.getConfigurator().apply(config.Slot0.withKS(newS)));
		DogLog.tunable(key + "/kV", config.Slot0.kV,
				newV -> motor.getConfigurator().apply(config.Slot0.withKV(newV)));
		DogLog.tunable(key + "/kG", config.Slot0.kG,
				newG -> motor.getConfigurator().apply(config.Slot0.withKG(newG)));
	}
}
