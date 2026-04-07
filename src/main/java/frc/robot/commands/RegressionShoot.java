package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.lib.util.Leds;
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

	private boolean aligned = false;

	public RegressionShoot(Swerve swerve, Shooter shooter, Indexer indexer, Intake intake, Supplier<Double> x,
			Supplier<Double> y) {
		super(shooter, indexer, intake);

		this.swerve = swerve;
		this.x = () -> x.get() / 5.0;
		this.y = () -> y.get() / 5.0;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		aligned = false;
		Leds.getInstance().shooting = true;
	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);
		boolean isAngledTowardsHub = !swerve.getYaw().getMeasure().isNear(params.angle().getMeasure(),
				params.maxAngleError());
		boolean areSticksMoving = Math.hypot(x.get(), y.get()) > 0;
		if (!params.isValid())
			return;

		runShoot(params.shooterRPS(), params.spinnerRPS(),
				swerve::atRotation);

		if (aligned == false) {
			swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);
		} else {
			swerve.setControl(new SwerveRequest.SwerveDriveBrake());
		}

		if (swerve.atRotation()) {
			aligned = true;
		}

		if (!isAngledTowardsHub || areSticksMoving) {
			aligned = false;
		}
	}
}
