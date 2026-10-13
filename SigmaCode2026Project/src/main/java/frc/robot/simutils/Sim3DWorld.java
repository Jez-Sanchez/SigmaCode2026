package frc.robot.simutils;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.simutils.FieldBuilder.Dims;
import org.littletonrobotics.junction.Logger;

/**
 * Top-level owner of the 3D simulation. Create it only in simulation, call {@link #periodic()} from
 * {@code Robot.simulationPeriodic()}, and nothing else in your robot code needs to know Jolt
 * exists.
 *
 * <p>For now it spawns two test boxes so you can confirm the field and the pose logging in
 * AdvantageScope. Remove them when RobotBody replaces them.
 */
public final class Sim3DWorld implements AutoCloseable {

  /** Matches the WPILib main loop. Fixed, so runs are reproducible. */
  private static final float LOOP_PERIOD_SECONDS = 0.02f;

  /** Jolt collision sub-steps per loop: 5 x 4 ms. Raise this if contacts look unstable. */
  private static final int COLLISION_STEPS = 5;

  /** Temporary: drop a couple of boxes so there is something to see. */
  private static final boolean SPAWN_TEST_BOXES = false;

  private final PhysicsWorld world;
  private final RobotBody robot;

  private int floorBoxId = -1;
  private int bumpBoxId = -1;

  public Sim3DWorld() {
    world = new PhysicsWorld(4096);
    FieldBuilder.build(world);

    // Start on open floor on the blue side, facing +X toward the bump
    robot = new RobotBody(world, RobotBody.Config.defaults(), 2.0, Dims.FIELD_WIDTH / 2.0, 0.0);

    if (SPAWN_TEST_BOXES) {
      // One box dropped onto open floor, one dropped onto the first bump's flat top
      floorBoxId = spawnTestBox(2.0, Dims.FIELD_WIDTH / 2.0, 2.0);
      bumpBoxId = spawnTestBox(Dims.BUMP_CENTER_X, Dims.BUMP_CENTER_Y, Dims.BUMP_HEIGHT + 2.0);
    }
  }

  // ---------------------------------------------------------------- per-loop update

  /** Step physics once and publish everything to the log. Call from simulationPeriodic(). */
  public void periodic() {
    robot.applyForces(); // forces must be applied before the step that consumes them
    world.step(LOOP_PERIOD_SECONDS, COLLISION_STEPS);
    log();
  }

  private void log() {
    BodyInterface bi = world.getBodyInterface();
    Logger.recordOutput("Sim3d/RobotPose3d", robot.getPose3d());

    if (SPAWN_TEST_BOXES) {
      Logger.recordOutput("Sim3d/FloorBoxPose", toPose3d(bi, floorBoxId));
      Logger.recordOutput("Sim3d/BumpBoxPose", toPose3d(bi, bumpBoxId));
    }

    // Static markers so you can check element placement in the 3D view
    Logger.recordOutput("Sim3d/BumpCenters", bumpCenters());
    Logger.recordOutput("Sim3d/TrenchCenters", trenchCenters());
  }

  // ---------------------------------------------------------------- conversions

  /** Read a body's pose out of Jolt as a WPILib Pose3d. */
  static Pose3d toPose3d(BodyInterface bi, int bodyId) {
    RVec3 p = bi.getPosition(bodyId);
    Quat q = bi.getRotation(bodyId);

    // Jolt's quaternion order is (x, y, z, w). WPILib's Quaternion constructor is (w, x, y, z).
    // Mixing these up is a silent bug, so the order is spelled out here.
    // VERIFY: Quat accessor names (getX/getY/getZ/getW) in your jolt-jni version.
    edu.wpi.first.math.geometry.Quaternion wpiQuat =
        new edu.wpi.first.math.geometry.Quaternion(q.getW(), q.getX(), q.getY(), q.getZ());

    return new Pose3d(new Translation3d(p.xx(), p.yy(), p.zz()), new Rotation3d(wpiQuat));
  }

  // ---------------------------------------------------------------- helpers

  /** Dynamic 0.5 m cube at the given position, for visual checks. Returns its body id. */
  private int spawnTestBox(double x, double y, double z) {
    BoxShape shape = new BoxShape(new Vec3(0.25f, 0.25f, 0.25f));
    BodyCreationSettings bcs =
        new BodyCreationSettings(
            shape, new RVec3(x, y, z), Quat.sIdentity(), EMotionType.Dynamic, LayerConstants.ROBOT);
    return world.addBody(bcs, EActivation.Activate);
  }

  private static Pose3d[] bumpCenters() {
    return mirrored4(Dims.BUMP_CENTER_X, Dims.BUMP_CENTER_Y, Dims.BUMP_HEIGHT);
  }

  private static Pose3d[] trenchCenters() {
    return mirrored4(Dims.TRENCH_CENTER_X, Dims.TRENCH_CENTER_Y, Dims.TRENCH_CLEARANCE);
  }

  /** The same four-corner mirroring FieldBuilder uses: (x,y), (L-x,y), (x,W-y), (L-x,W-y). */
  private static Pose3d[] mirrored4(double x, double y, double z) {
    double mx = Dims.FIELD_LENGTH - x;
    double my = Dims.FIELD_WIDTH - y;
    return new Pose3d[] {
      new Pose3d(x, y, z, new Rotation3d()),
      new Pose3d(mx, y, z, new Rotation3d()),
      new Pose3d(x, my, z, new Rotation3d()),
      new Pose3d(mx, my, z, new Rotation3d())
    };
  }

  /** The robot chassis. Use this to command speeds and read its 3D pose. */
  public RobotBody robot() {
    return robot;
  }

  /** Teleport the robot to a 2D pose, level and at rest. */
  public void resetRobot(Pose2d pose) {
    robot.reset(pose);
  }

  /** Access for the classes that will be added next (game pieces, intakes). */
  public PhysicsWorld getWorld() {
    return world;
  }

  @Override
  public void close() {
    world.close();
  }
}
