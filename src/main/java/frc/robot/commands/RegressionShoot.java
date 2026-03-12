package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
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
		this.x = x;
		this.y = y;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);
		runShoot(params.shooterRPS(), params.spinnerRPS(),
				swerve::atRotation);

		boolean usingSticks = Math.hypot(x.get(), y.get()) > 0.1;
		boolean isWithinHub = swerve.getYaw().getMeasure().isNear(params.angle().getMeasure(),
				params.maxAngleError());

		if (!aligned) {
			swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);
		} else {
			swerve.setControl(new SwerveRequest.SwerveDriveBrake());
		}
		// keep x configuration so long as we won't theoretically miss half our shots
		// from our angle
		if (usingSticks || !isWithinHub) {
			aligned = false;
		}

		if (swerve.atRotation()) {
			aligned = true;
		}

	}
}
