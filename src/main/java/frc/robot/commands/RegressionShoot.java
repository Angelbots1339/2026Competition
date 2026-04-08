package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Leds;
import frc.robot.Constants.ShootingConstants;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RegressionShoot extends Shoot {
	private Swerve swerve;
	private Shooter shooter;

	private Supplier<Double> x;
	private Supplier<Double> y;

	private boolean aligned = false;

	public RegressionShoot(Swerve swerve, Shooter shooter, Indexer indexer, Intake intake, Supplier<Double> x,
			Supplier<Double> y) {
		super(shooter, indexer, intake);

		this.shooter = shooter;

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
		boolean isAngledTowardsHub = swerve.getYaw().getMeasure().isNear(params.angle().getMeasure(),
				params.maxAngleError());
		boolean areSticksMoving = Math.hypot(x.get(), y.get()) > 0;
		DogLog.log("Regression Shoot/Sticks", areSticksMoving);
		DogLog.log("Regression Shoot/Angled At Hub", isAngledTowardsHub);
		if (!params.isValid())
			return;

		if (swerve.getTotalStatorCurrent() <= ShootingConstants.MaximumOtherCurrentDraw) {
			runShoot(params.shooterRPS(), params.spinnerRPS(),
					swerve::atRotation);
		}

		if (aligned == false) {
			Leds.getInstance().shooterMisaligned = true;
			swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);
			shooter.disableKicker();
		} else {
			Leds.getInstance().shooterMisaligned = false;
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
