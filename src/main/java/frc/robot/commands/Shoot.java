package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
	private Swerve swerve;
	private Shooter shooter;
	private Intake intake;

	private Supplier<Double> x;
	private Supplier<Double> y;
	private Supplier<Boolean> runIndex;

	private Timer intakeTimer = new Timer();

	public Shoot(Swerve swerve, Shooter shooter, Intake intake, Supplier<Double> x, Supplier<Double> y,
			Supplier<Boolean> runIndex) {
		this.swerve = swerve;
		this.shooter = shooter;
		this.intake = intake;

		this.x = x;
		this.y = y;
		this.runIndex = runIndex;
		addRequirements(shooter, swerve, intake);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);
		swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);

		shooter.setRPS(params.shooterRPS(), params.spinnerRPS());

		if (shooter.atSetpoint() && runIndex.get()) {
			intakeTimer.start();
			shooter.runIndexVelocity(20);
			intake.setIntakeVoltage(IntakeConstants.IntakeVoltage);
			if (intakeTimer.hasElapsed(0.25)) {
				intake.setIntakeAngle(IntakeConstants.DeployedAngle.plus(Degrees.of(30)));
			}
			if (intakeTimer.hasElapsed(0.5)) {
				intake.setIntakeAngle(IntakeConstants.DeployedAngle);
				intakeTimer.reset();
			}
		}

		// if (!runIndex.get() ||
		// swerve.getRotationError().minus(params.angle()).getMeasure()
		// .gt(params.maxAngleError()))
		// shooter.disableIndex();
	}

	@Override
	public void end(boolean interrupted) {
		shooter.disableIndex();
		shooter.disableShooter();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
