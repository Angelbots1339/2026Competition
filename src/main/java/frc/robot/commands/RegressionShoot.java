package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RegressionShoot extends Shoot {
	private Swerve swerve;

	private Supplier<Double> x;
	private Supplier<Double> y;

	public RegressionShoot(Swerve swerve, Shooter shooter, Indexer indexer, Intake intake, Supplier<Double> x,
			Supplier<Double> y) {
		super(shooter, indexer, intake);

		this.swerve = swerve;
		this.x = x;
		this.y = y;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);
		swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);
		runShoot(params.shooterRPS(), params.spinnerRPS(),
				() -> true);
		// () ->
		// swerve.getRotationError().getMeasure().isNear(params.angle().getMeasure(),
		// params.maxAngleError()));
	}
}
