package simutils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.simutils.FieldBuilder;
import frc.robot.simutils.FieldBuilder.Dims;
import frc.robot.simutils.PhysicsWorld;
import frc.robot.simutils.RobotBody;
import frc.robot.simutils.SwerveDriveSim;
import frc.robot.simutils.SwerveModuleSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Tests for the raycast swerve model. They use the same stepping as Sim3dWorld: five 4 ms sub-steps
 * per 20 ms loop, applying wheel forces before each one.
 *
 * <p>The constants below duplicate the ones in Sim3dWorld. Keep them in sync.
 *
 * <p>VERIFY: the Vec3 accessor in {@link Rig#velocityX} (getX() vs xx()).
 */
class SwerveDriveSimTest {

  private static final double LOOP_DT = 0.02;
  private static final int SUB_STEPS = 5;
  private static final double GROUND_CLEARANCE_M = 0.05;
  private static final double MODULE_HALF_SPAN_M = 0.32;

  /**
   * DCMotorSim clamps its input to RobotController.getBatteryVoltage(), which calls into the WPILib
   * HAL. A real robot or sim run has already initialized it, but a bare JUnit test has not, and
   * touching the HAL then crashes the whole JVM with an access violation in wpiHal.dll.
   */
  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "WPILib HAL failed to initialize");
  }

  /** A field, a robot, and a swerve drive, with one-line stepping. */
  private static final class Rig implements AutoCloseable {
    final PhysicsWorld world = new PhysicsWorld(1024);
    final RobotBody robot;
    final SwerveDriveSim swerve;
    final double massKg;

    Rig(double x, double y, double yawRad) {
      FieldBuilder.build(world);
      RobotBody.Config cfg = RobotBody.Config.defaults();
      massKg = cfg.massKg();
      robot = new RobotBody(world, cfg, x, y, yawRad, GROUND_CLEARANCE_M);
      swerve =
          new SwerveDriveSim(
              world,
              robot,
              cfg.massKg(),
              SwerveDriveSim.squareMounts(MODULE_HALF_SPAN_M),
              SwerveModuleSim.Config.defaults());
    }

    /** One 20 ms loop. */
    void step() {
      double subDt = LOOP_DT / SUB_STEPS;
      for (int i = 0; i < SUB_STEPS; i++) {
        swerve.update(subDt);
        world.step((float) subDt, 1);
      }
    }

    void run(double seconds) {
      int loops = (int) Math.round(seconds / LOOP_DT);
      for (int i = 0; i < loops; i++) {
        step();
      }
    }

    Pose3d pose() {
      return robot.getPose3d();
    }

    double velocityX() {
      return world.getBodyInterface().getLinearVelocity(robot.getBodyId()).getX();
    }

    @Override
    public void close() {
      world.close();
    }
  }

  // ---------------------------------------------------------------- resting on the wheels

  /**
   * With no commands, the wheels alone must hold the robot at floor level, level, with the springs
   * carrying the whole weight.
   */
  @Test
  void restsOnWheels() {
    try (Rig rig = new Rig(2.0, Dims.FIELD_WIDTH / 2.0, 0.0)) {
      rig.swerve.setVoltages(new double[4], new double[4]);
      rig.run(1.5);

      Pose3d pose = rig.pose();
      assertEquals(0.0, pose.getZ(), 0.02, "body origin should settle at floor level");
      assertEquals(0.0, pose.getRotation().getX(), 0.03, "roll");
      assertEquals(0.0, pose.getRotation().getY(), 0.03, "pitch");

      for (int i = 0; i < rig.swerve.getModuleCount(); i++) {
        assertTrue(rig.swerve.isModuleGrounded(i), "module " + i + " should be touching the floor");
      }

      double weight = rig.massKg * 9.81;
      assertEquals(
          weight,
          rig.swerve.getTotalNormalForce(),
          0.1 * weight,
          "wheel normal forces should add up to the robot's weight");
    }
  }

  // ---------------------------------------------------------------- driving

  /** A forward command should bring the robot to about that speed, in a straight line. */
  @Test
  void drivesForwardAtCommandedSpeed() {
    double startY = Dims.FIELD_WIDTH / 2.0;
    try (Rig rig = new Rig(2.0, startY, 0.0)) {
      rig.swerve.setChassisSpeedsCommand(new ChassisSpeeds(1.5, 0.0, 0.0));
      rig.run(2.0);

      Pose3d pose = rig.pose();
      assertEquals(1.5, rig.velocityX(), 0.2, "should reach the commanded speed");
      assertTrue(pose.getX() > 3.5, "should have moved forward, x = " + pose.getX());
      assertEquals(startY, pose.getY(), 0.1, "should not drift sideways");
      assertEquals(0.0, pose.getRotation().getZ(), 0.1, "should not rotate");
    }
  }

  /** A sideways command must steer every module to about 90 degrees and move the robot in +Y. */
  @Test
  void modulesSteerToCommand() {
    double startY = Dims.FIELD_WIDTH / 2.0;
    try (Rig rig = new Rig(2.0, startY, 0.0)) {
      rig.swerve.setChassisSpeedsCommand(new ChassisSpeeds(0.0, 1.0, 0.0));
      rig.run(1.5);

      for (int i = 0; i < rig.swerve.getModuleCount(); i++) {
        double cos = rig.swerve.getSteerAngle(i).getCos();
        assertEquals(0.0, cos, 0.2, "module " + i + " should point sideways");
      }
      assertTrue(
          rig.pose().getY() > startY + 0.4,
          "should have strafed toward +Y, y = " + rig.pose().getY());
    }
  }

  // ---------------------------------------------------------------- the point of the project

  /**
   * Driving straight over the bump: the robot must make it across, pitch up and down while doing
   * so, and end up level on the floor again.
   *
   * <p>This is the test most likely to need tuning, since it depends on the placeholder bump
   * dimensions, the ground clearance, and the wheel spring settings.
   */
  @Test
  void climbsOverBump() {
    double bumpStart = Dims.BUMP_CENTER_X - Dims.BUMP_TOTAL_LENGTH / 2.0;
    double bumpEnd = Dims.BUMP_CENTER_X + Dims.BUMP_TOTAL_LENGTH / 2.0;

    try (Rig rig = new Rig(bumpStart - 1.2, Dims.BUMP_CENTER_Y, 0.0)) {
      rig.swerve.setChassisSpeedsCommand(new ChassisSpeeds(1.5, 0.0, 0.0));

      double maxPitch = 0.0;
      int loops = (int) Math.round(4.0 / LOOP_DT);
      for (int i = 0; i < loops; i++) {
        rig.step();
        maxPitch = Math.max(maxPitch, Math.abs(rig.pose().getRotation().getY()));
      }

      Pose3d pose = rig.pose();
      assertTrue(pose.getX() > bumpEnd + 0.3, "robot should be past the bump, x = " + pose.getX());
      assertTrue(maxPitch > 0.05, "robot should pitch over the bump, max pitch = " + maxPitch);
      assertTrue(maxPitch < 0.6, "robot pitched too violently, max pitch = " + maxPitch);
      assertEquals(0.0, pose.getZ(), 0.03, "robot should be back on the floor");
    }
  }
}
