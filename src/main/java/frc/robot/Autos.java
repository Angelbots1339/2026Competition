package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.choreo.ChoreoTraj;
import frc.lib.util.FieldUtil;
import frc.robot.commands.AutoShoot;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Autos {
	private AutoFactory factory;

	Supplier<Command> shoot = null;

	public Autos(Swerve swerve, Shooter shooter, Intake intake, Indexer indexer) {
		factory = new AutoFactory(
				swerve::getPose, // A function that returns the current field-relative Pose2d of the robot
				swerve::resetPose, // A function that receives a field-relative Pose2d to reset the robot's
									// odometry to.
				swerve.followChoreoPath(), // A function that receives the current SampleType and controls the
											// robot. More info below
				true, // If this is true, when on the red alliance, the path will be mirrored to the
						// opposite side, while keeping the same coordinate system origin.
				swerve); // The drive Subsystem to require for AutoTrajectory Commands.

		shoot = () -> new AutoShoot(swerve, shooter, indexer, intake, () -> 0.0, () -> 0.0);

		factory.bind("RevUpShooter", shooter.run(() -> {
			ShooterParams params = ShooterRegression.getShotParams(swerve);
			shooter.setRPS(params.shooterRPS(), params.spinnerRPS());
		}));
		factory.bind("IntakeStart", intake.runIntake());
		factory.bind("IntakeStop", intake.stopIntake());
		CommandScheduler.getInstance().schedule(factory.trajectoryCmd("").ignoringDisable(true));
	}

	public AutoRoutine rightNeutral() {
		final var routine = factory.newRoutine("Right Neutral");
		final var bumpToNeutral = routine
				.trajectory(
						flipTrajectoryX(routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.Shoot_To_Neutral.name()).getRawTrajectory()));
		final var NeutralSend = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name()).getRawTrajectory()));

		final var shoot1 = shoot.get().withTimeout(3.5);
		final var shoot2 = shoot.get().withTimeout(3.5);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine rightNeutralSweep() {
		final var routine = factory.newRoutine("Right Neutral Sweep");
		final var bumpToNeutral = routine
				.trajectory(
						flipTrajectoryX(routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.Shoot_To_HubSweep.name()).getRawTrajectory()));
		final var NeutralSend = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name()).getRawTrajectory()));

		final var shoot1 = shoot.get().withTimeout(3.1);
		final var shoot2 = shoot.get().withTimeout(3);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	private Trajectory<SwerveSample> flipTrajectoryX(Trajectory<SwerveSample> traj) {
		SwerveSample[] new_samples = new SwerveSample[traj.samples().size()];
		int i = 0;
		for (SwerveSample sample : traj.samples()) {
			new_samples[i++] = new SwerveSample(sample.t, sample.x, FieldUtil.FieldHeight.in(Meters) - sample.y,
					-sample.heading,
					sample.vx, -sample.vy,
					-sample.omega, sample.ax, -sample.ay, -sample.alpha, sample.moduleForcesX(), new double[] {
							-sample.moduleForcesY()[1],
							-sample.moduleForcesY()[0],
							-sample.moduleForcesY()[3],
							-sample.moduleForcesY()[2]
					});
		}

		return new Trajectory<SwerveSample>(traj.name(), List.of(new_samples), traj.splits(), traj.events());
	}
}
