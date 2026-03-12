package frc.robot.commands;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private Intake intake;

	private Supplier<Double> shooterRPS;
	private Supplier<Double> spinnerRPS;

	private Supplier<Boolean> runKicker = () -> true;

	private Timer cycleTimer = new Timer();

	public Shoot(Shooter shooter, Indexer indexer, Intake intake) {
		this(shooter, indexer, intake, () -> 0.0, () -> 0.0, () -> true);
	}

	public Shoot(Shooter shooter, Indexer indexer, Intake intake, Supplier<Double> shooterRPS,
			Supplier<Double> spinnerRPS,
			Supplier<Boolean> runKicker) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.intake = intake;
		this.shooterRPS = shooterRPS;
		this.spinnerRPS = spinnerRPS;
		this.runKicker = runKicker;

		addRequirements(shooter, indexer, intake);
	}

	@Override
	public void initialize() {
		cycleTimer.reset();
	}

	public void runShoot(double shooterRPS, double spinnerRPS, Supplier<Boolean> runKicker) {
		shooter.setRPS(shooterRPS, spinnerRPS);

		if (shooter.atSetpoint() && runKicker.get()) {
			shooter.setKickerVelocity(ShooterConstants.KickerRPS);
			indexer.runVoltage(IndexerConstants.IndexerVolts);
			intake.setIntakeVoltage(IntakeConstants.IntakeVoltage / 2.0);

			if (!cycleTimer.isRunning())
				cycleTimer.restart();

			if (cycleTimer.hasElapsed(0.5)) {
				intake.setIntakeAngle(Degrees.of(
						MathUtil.interpolate(IntakeConstants.DeployedAngle.in(Degrees),
								IntakeConstants.RetractedAngle.in(Degrees),
								(1.0 / ShootingConstants.IntakeRetractTime.in(Seconds)) * (cycleTimer.get() - 0.5))));
			}

			// if (cycleTimer.hasElapsed(0.5)) {
			// intake.setIntakeAngle(IntakeConstants.AgitationAngle2);
			// }
			// if (cycleTimer.hasElapsed(1.5)) {
			// intake.setIntakeAngle(IntakeConstants.DeployedAngle);
			// }
			if (cycleTimer.hasElapsed(ShootingConstants.IntakeRetractTime.in(Seconds))) {
				cycleTimer.restart();
			}
		}
	}

	@Override
	public void execute() {
		runShoot(shooterRPS.get(), spinnerRPS.get(), runKicker);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.disableKicker();
		shooter.disableShooter();
		indexer.disable();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
