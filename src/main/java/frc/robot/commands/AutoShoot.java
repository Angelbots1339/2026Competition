package frc.robot.commands;

import java.util.function.Supplier;

import frc.lib.util.Leds;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// this is basically just RegressionShoot but without the X locking mechanism
public class AutoShoot extends Shoot {
	private Swerve swerve;

	private Supplier<Double> x;
	private Supplier<Double> y;

	public AutoShoot(Swerve swerve, Shooter shooter, Indexer indexer, Intake intake, Supplier<Double> x,
			Supplier<Double> y) {
		super(shooter, indexer, intake);

		this.swerve = swerve;
		this.x = () -> x.get() / 5.0;
		this.y = () -> y.get() / 5.0;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		Leds.getInstance().shooting = true;
	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);

		runShoot(params.shooterRPS(), params.spinnerRPS(),
				swerve::atRotation);
		swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);
	}
}
