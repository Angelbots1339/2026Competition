package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.choreo.ChoreoTraj;
import frc.lib.util.FieldUtil;
import frc.robot.commands.Shoot;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
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

		shoot = () -> new Shoot(swerve, shooter, indexer, () -> 0.0, () -> 0.0, () -> true)
				.withTimeout(4);

		factory.bind("IntakeStart", intake.runIntake().alongWith(indexer.index()));
		factory.bind("IntakeStop", intake.stopIntake().alongWith(indexer.run(() -> indexer.disable())));
		factory.bind("RevUpShooter", shooter.run(() -> {
			ShooterParams params = ShooterRegression.getShotParams(swerve);
			shooter.setRPS(params.shooterRPS(), params.spinnerRPS());
		}));
	}

	public Command hubDepotOutpostTowerAuto() {
		final var routine = factory.newRoutine("Hub Depot Outpost Tower");
		final var hubToDepotShoot = routine.trajectory(ChoreoTraj.HubtoDepotShoot.name());
		final var depotShootToOutpostShoot = routine.trajectory(ChoreoTraj.DepotShootOutpostShoot.name());

		final var shoot1 = shoot.get();
		final var shoot2 = shoot.get();

		routine.active().onTrue(
				Commands.sequence(
						hubToDepotShoot.resetOdometry(),
						hubToDepotShoot.cmd()));
		hubToDepotShoot.done().onTrue(shoot1);

		routine.observe(shoot1::isFinished).onTrue(depotShootToOutpostShoot.cmd());
		depotShootToOutpostShoot.done().onTrue(shoot2);

		return routine.cmd();
	}

	public AutoRoutine rightNeutral() {
		final var routine = factory.newRoutine("Right Neutral");
		final var bumpToNeutral = routine
				.trajectory(flipTrajectoryX(routine.trajectory(ChoreoTraj.BumpToNeutral.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.DepotShootNeutral2.name()).getRawTrajectory()));

		final var shoot1 = shoot.get();
		final var shoot2 = shoot.get();

		routine.active().onTrue(
				Commands.sequence(
						bumpToNeutral.resetOdometry(),
						bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);

		return routine;
	}

	public AutoRoutine leftNeutral() {
		final var routine = factory.newRoutine("Left Neutral");
		final var bumpToNeutral = routine.trajectory(ChoreoTraj.BumpToNeutral.name());
		final var neutral2 = routine.trajectory(ChoreoTraj.DepotShootNeutral2.name());

		final var shoot1 = shoot.get();
		final var shoot2 = shoot.get();

		routine.active().onTrue(
				Commands.sequence(
						bumpToNeutral.resetOdometry(),
						bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(neutral2.cmd());
		neutral2.done().onTrue(shoot2);

		return routine;
	}

	public Command leftDepotNeutral() {
		final var routine = factory.newRoutine("Left Depot Neutral");
		final var leftDepot = routine.trajectory(ChoreoTraj.HubtoDepotShoot.name());
		final var leftNeutral1 = routine.trajectory(ChoreoTraj.DepotShootNeutral1.name());
		final var leftNeutral2 = routine.trajectory(ChoreoTraj.DepotShootNeutral2.name());

		final var shoot1 = shoot.get();
		final var shoot2 = shoot.get();
		final var shoot3 = shoot.get();

		routine.active().onTrue(
				Commands.sequence(
						leftDepot.resetOdometry(),
						leftDepot.cmd()));
		leftDepot.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral1.cmd());
		leftNeutral1.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot3);

		return routine.cmd();
	}

	public Command rightOutpostNeutral() {
		final var routine = factory.newRoutine("Right Outpost Neutral");
		final var rightOutpost = routine.trajectory(ChoreoTraj.RightOutpostShoot.name());
		final var rightNeutral1 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.DepotShootNeutral1.name()).getRawTrajectory()));
		final var rightNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.DepotShootNeutral2.name()).getRawTrajectory()));

		final var shoot1 = shoot.get();
		final var shoot2 = shoot.get();
		final var shoot3 = shoot.get();

		routine.active().onTrue(
				Commands.sequence(
						rightOutpost.resetOdometry(),
						rightOutpost.cmd()));
		rightOutpost.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(rightNeutral1.cmd());
		rightNeutral1.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(rightNeutral2.cmd());
		rightNeutral2.done().onTrue(shoot3);

		return routine.cmd();
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
