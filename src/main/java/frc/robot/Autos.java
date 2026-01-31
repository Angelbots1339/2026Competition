package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.choreo.ChoreoTraj;
import frc.lib.util.FieldUtil;
import frc.robot.subsystems.Swerve;

public class Autos {
	private AutoFactory factory;
	private Swerve swerve;

	public Autos(Swerve swerve) {
		this.swerve = swerve;
		factory = new AutoFactory(
				swerve::getPose, // A function that returns the current field-relative Pose2d of the robot
				swerve::resetPose, // A function that receives a field-relative Pose2d to reset the robot's
									// odometry to.
				swerve.followChoreoPath(), // A function that receives the current SampleType and controls the
											// robot. More info below
				true, // If this is true, when on the red alliance, the path will be mirrored to the
						// opposite side, while keeping the same coordinate system origin.
				swerve); // The drive Subsystem to require for AutoTrajectory Commands.

		/* TODO: replace with actual commands */
		Command leftIntakeOpen = Commands.print("intake open").andThen(Commands.waitSeconds(1));
		Command leftIntakeClose = Commands.print("intake close").andThen(Commands.waitSeconds(1));

		Command passStart = Commands.print("pass start");
		Command passEnd = Commands.print("pass end");

		factory.bind("LeftIntakeStart", leftIntakeOpen);
		factory.bind("LeftIntakeEnd", leftIntakeClose);
		factory.bind("PassStart", passStart);
		factory.bind("PassEnd", passEnd);
	}

	public Command bumpTest() {
		final var routine = factory.newRoutine("bump test");
		final var bumptest = routine.trajectory(ChoreoTraj.BumpTest.name());
		routine.active().onTrue(
				Commands.sequence(
						bumptest.resetOdometry(),
						bumptest.cmd(),
						swerve.pointDriveCommand(() -> 0.0, () -> 0.0, () -> FieldUtil.getHubCenter(), () -> true)));

		return routine.cmd();
	}

	public Command leftPassAuto() {
		final var routine = factory.newRoutine("left pass");
		final var leftPass = routine.trajectory(ChoreoTraj.LeftPass.name());

		routine.active().onTrue(
				Commands.sequence(
						leftPass.resetOdometry(),
						leftPass.cmd()));

		return routine.cmd();
	}

	public Command hubDepotTowerAuto() {
		final var routine = factory.newRoutine("Hub Depot Tower");
		final var hubToDepotShoot = routine.trajectory(ChoreoTraj.HubtoDepotShoot.name());
		final var depotShoottoTower = routine.trajectory(ChoreoTraj.DepotShoottoTower.name());

		routine.active().onTrue(
				Commands.sequence(
						hubToDepotShoot.resetOdometry(),
						hubToDepotShoot.cmd(),
						depotShoottoTower.cmd()));

		return routine.cmd();
	}

	public Command hubDepotOutpostTowerAuto() {
		final var routine = factory.newRoutine("Hub Depot Outpost Tower");
		final var hubToDepotShoot = routine.trajectory(ChoreoTraj.HubtoDepotShoot.name());
		final var depotShoottoOutpost = routine.trajectory(ChoreoTraj.DepotShoottoOutpost.name());
		final var outpostToShoot = routine.trajectory(ChoreoTraj.OutposttoShoot.name());
		final var outpostShoottoTower = routine.trajectory(ChoreoTraj.OutpostShoottoTower.name());

		routine.active().onTrue(
				Commands.sequence(
						hubToDepotShoot.resetOdometry(),
						hubToDepotShoot.cmd(),
						depotShoottoOutpost.cmd(),
						outpostToShoot.cmd(),
						outpostShoottoTower.cmd()));

		return routine.cmd();
	}

	public Command leftNeutralAuto() {
		final var routine = factory.newRoutine("Left Neutral");
		final var leftNeutralToShoot = routine.trajectory(ChoreoTraj.LeftNeutralToShoot.name());

		routine.active().onTrue(
				Commands.sequence(
						leftNeutralToShoot.resetOdometry(),
						leftNeutralToShoot.cmd()));
		return routine.cmd();
	}

	public Command rightNeutralAuto() {
		final var routine = factory.newRoutine("Right Neutral");
		final var rightNeutralToShoot = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.LeftNeutralToShoot.name()).getRawTrajectory()));

		routine.active().onTrue(
				Commands.sequence(
						rightNeutralToShoot.resetOdometry(),
						rightNeutralToShoot.cmd()));
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
