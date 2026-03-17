// spotless:off
package frc.lib.choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity BumpPassVel = Units.MetersPerSecond.of(2);
    public static final Distance FieldHeight = Units.Meters.of(8.069326);
    public static final LinearVelocity IntakeVelocity = Units.MetersPerSecond.of(1);

    public static final class Poses {
        public static final Pose2d DepotIntake = new Pose2d(0.75, 5.0020819, Rotation2d.fromRadians(2.2340214));
        public static final Pose2d DepotIntakeEnd = new Pose2d(0.75, 6.71, Rotation2d.fromRadians(2.2340214));
        public static final Pose2d DepotShoot = new Pose2d(2.3219109, 5.3971691, Rotation2d.fromRadians(-0.5880032));
        public static final Pose2d FirstBumpStart = new Pose2d(3.7402761, 5.8216133, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d HubLeftStart = new Pose2d(3.5654624, 5.0397372, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d LeftBumpStart = new Pose2d(3.6429224, 6.0623264, Rotation2d.fromRadians(-3.1415927));
        public static final Pose2d LeftNeutralFarmStop = new Pose2d(7.7637601, 5.0096962, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d LeftNeutralStart = new Pose2d(7.7637601, 6.8411598, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d LeftNeutralStop = new Pose2d(7.7637601, 5.5309696, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d NeutralShoot = new Pose2d(3.4846222, 5.4584527, Rotation2d.fromRadians(2.2367652));
        public static final Pose2d SecondBumpStart = new Pose2d(5.6032634, 5.562218, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d SecondBumpStop = new Pose2d(3.65137, 5.562218, Rotation2d.fromRadians(-1.5707963));
    }
}
// spotless:on
