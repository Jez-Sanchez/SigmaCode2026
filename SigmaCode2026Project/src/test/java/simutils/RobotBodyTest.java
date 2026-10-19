package simutils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.simutils.FieldBuilder;
import frc.robot.simutils.FieldBuilder.Dims;
import frc.robot.simutils.PhysicsWorld;
import frc.robot.simutils.RobotBody;
import org.junit.jupiter.api.Test;

/**
 * Tests for the force-driven robot chassis. They use the same stepping as Sim3dWorld: apply forces,
 * then step 20 ms in 5 sub-steps.
 *
 * <p>VERIFY: the Vec3 accessor used in {@link #velocityX} (getX() vs xx()), matching whatever your
 * RobotBody compiles with.
 */
class RobotBodyTest {

  private static final float DT = 0.02f;
  private static final int COLLISION_STEPS = 5;

  // ---------------------------------------------------------------- helpers

  /** Default config with a different total height. */
  private static RobotBody.Config configWithHeight(double robotHeightM) {
    RobotBody.Config d = RobotBody.Config.defaults();
    return new RobotBody.Config(
        d.massKg(),
        d.lengthM(),
        d.widthM(),
        d.chassisHeightM(),
        robotHeightM,
        d.superLengthM(),
        d.superWidthM(),
        d.maxAccelMps2(),
        d.maxAngularAccelRadPerSec2(),
        d.velocityTimeConstantSec(),
        d.friction());
  }

  /** Apply forces then step, the same order as Sim3dWorld.periodic(). */
  private static void run(PhysicsWorld world, RobotBody robot, double seconds) {
    int steps = (int) Math.round(seconds / DT);
    for (int i = 0; i < steps; i++) {
      robot.applyForces();
      world.step(DT, COLLISION_STEPS);
    }
  }

  private static double velocityX(PhysicsWorld world, RobotBody robot) {
    return world.getBodyInterface().getLinearVelocity(robot.getBodyId()).getX();
  }

  private static double velocityY(PhysicsWorld world, RobotBody robot) {
    return world.getBodyInterface().getLinearVelocity(robot.getBodyId()).getY();
  }

  // ---------------------------------------------------------------- resting pose

  /**
   * The body origin must sit on the floor and the robot must rest level, whatever its height. This
   * is the "floating robot" regression check: a tall robot used to sit with its origin at half its
   * height.
   */
  @Test
  void restsOnFloorAtAnyHeight() {
    double[] heights = {0.25, 0.5, 0.7, 1.0}; // 0.25 means no superstructure box at all
    for (double h : heights) {
      try (PhysicsWorld world = new PhysicsWorld(1024)) {
        FieldBuilder.build(world);
        RobotBody robot =
            new RobotBody(world, configWithHeight(h), 2.0, Dims.FIELD_WIDTH / 2.0, 0.0);

        run(world, robot, 1.0);

        Pose3d pose = robot.getPose3d();
        assertEquals(0.0, pose.getZ(), 0.03, "origin should be on the floor at height " + h);
        assertEquals(0.0, pose.getRotation().getX(), 0.05, "roll at height " + h);
        assertEquals(0.0, pose.getRotation().getY(), 0.05, "pitch at height " + h);
      }
    }
  }

  // ---------------------------------------------------------------- driving

  /** A forward command should bring the robot to roughly that speed, in a straight line. */
  @Test
  void drivesForwardAtCommandedSpeed() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      double startX = 2.0;
      double startY = Dims.FIELD_WIDTH / 2.0;
      RobotBody robot = new RobotBody(world, RobotBody.Config.defaults(), startX, startY, 0.0);

      robot.setTargetSpeeds(new ChassisSpeeds(1.5, 0.0, 0.0));
      run(world, robot, 1.5);

      Pose3d pose = robot.getPose3d();
      assertEquals(1.5, velocityX(world, robot), 0.2, "should reach the commanded speed");
      assertTrue(pose.getX() > startX + 1.0, "should have moved forward, x = " + pose.getX());
      assertEquals(startY, pose.getY(), 0.1, "should not drift sideways");
      assertEquals(0.0, pose.getRotation().getZ(), 0.1, "should not rotate");
    }
  }

  /** A robot facing +Y given a robot-relative forward command must move along +Y. */
  @Test
  void robotRelativeCommandFollowsHeading() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      double startX = 2.0;
      double startY = Dims.FIELD_WIDTH / 2.0;
      RobotBody robot =
          new RobotBody(world, RobotBody.Config.defaults(), startX, startY, Math.PI / 2.0);

      robot.setTargetSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
      run(world, robot, 1.5);

      Pose3d pose = robot.getPose3d();
      assertTrue(pose.getY() > startY + 0.8, "should move toward +Y, y = " + pose.getY());
      assertEquals(startX, pose.getX(), 0.1, "should not move in X");
    }
  }

  /** A field-relative +X command must move the robot along +X even when it faces +Y. */
  @Test
  void fieldRelativeCommandIgnoresHeading() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      double startX = 2.0;
      double startY = Dims.FIELD_WIDTH / 2.0;
      RobotBody robot =
          new RobotBody(world, RobotBody.Config.defaults(), startX, startY, Math.PI / 2.0);

      robot.setTargetSpeedsFieldRelative(new ChassisSpeeds(1.0, 0.0, 0.0));
      run(world, robot, 1.5);

      Pose3d pose = robot.getPose3d();
      assertTrue(pose.getX() > startX + 0.8, "should move toward +X, x = " + pose.getX());
      assertEquals(startY, pose.getY(), 0.1, "should not move in Y");
    }
  }

  // ---------------------------------------------------------------- collisions

  /** Driving into the x = 0 wall must stop the robot with its edge against the wall. */
  @Test
  void wallStopsRobot() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      RobotBody robot =
          new RobotBody(world, RobotBody.Config.defaults(), 2.0, Dims.FIELD_WIDTH / 2.0, 0.0);

      robot.setTargetSpeeds(new ChassisSpeeds(-2.0, 0.0, 0.0)); // backward, toward x = 0
      run(world, robot, 3.0);

      double x = robot.getPose3d().getX();
      double halfLength = RobotBody.Config.defaults().lengthM() / 2.0;
      // The center cannot get closer to the wall than half the robot length
      assertTrue(x > halfLength - 0.1, "robot went through the wall, x = " + x);
      assertTrue(x < halfLength + 0.15, "robot stopped too far from the wall, x = " + x);
      assertEquals(0.0, velocityX(world, robot), 0.1, "robot should be stopped against the wall");
    }
  }

  // ---------------------------------------------------------------- reset

  /** reset() must put the robot at the requested pose, level and motionless. */
  @Test
  void resetPlacesRobotLevelAtRest() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      RobotBody robot =
          new RobotBody(world, RobotBody.Config.defaults(), 2.0, Dims.FIELD_WIDTH / 2.0, 0.0);
      robot.setTargetSpeeds(new ChassisSpeeds(1.5, 0.5, 1.0));
      run(world, robot, 1.0); // get it moving first

      robot.reset(new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(45)));

      Pose3d pose = robot.getPose3d();
      assertEquals(3.0, pose.getX(), 0.01);
      assertEquals(3.0, pose.getY(), 0.01);
      assertEquals(Math.toRadians(45), pose.getRotation().getZ(), 0.01);
      assertEquals(0.0, velocityX(world, robot), 1e-3);
      assertEquals(0.0, velocityY(world, robot), 1e-3);

      // Without a command it should stay put
      run(world, robot, 0.5);
      assertEquals(3.0, robot.getPose3d().getX(), 0.05, "reset should clear the old command");
      assertEquals(3.0, robot.getPose3d().getY(), 0.05, "reset should clear the old command");
    }
  }

  // ---------------------------------------------------------------- trench height

  /** Drives along +X into the trench from 1.5 m away and returns the robot's final X. */
  private static double driveIntoTrench(double robotHeightM) {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      double trenchStart = Dims.TRENCH_CENTER_X - Dims.TRENCH_LENGTH / 2.0;
      RobotBody robot =
          new RobotBody(
              world, configWithHeight(robotHeightM), trenchStart - 1.5, Dims.TRENCH_CENTER_Y, 0.0);

      robot.setTargetSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
      run(world, robot, 4.0);

      return robot.getPose3d().getX();
    }
  }

  /** A robot shorter than the clearance must drive all the way through. */
  @Test
  void shortRobotFitsUnderTrench() {
    double height = Dims.TRENCH_CLEARANCE - 0.05;
    double x = driveIntoTrench(height);

    double trenchEnd = Dims.TRENCH_CENTER_X + Dims.TRENCH_LENGTH / 2.0;
    assertTrue(x > trenchEnd, "Short robot should clear the trench, ended at x = " + x);
  }

  /** A robot taller than the clearance must be stopped by the beam. */
  @Test
  void tallRobotIsBlockedByTrench() {
    double height = Dims.TRENCH_CLEARANCE + 0.15;
    double x = driveIntoTrench(height);

    double trenchStart = Dims.TRENCH_CENTER_X - Dims.TRENCH_LENGTH / 2.0;
    assertTrue(x < trenchStart, "Tall robot should be blocked, ended at x = " + x);
  }
}
