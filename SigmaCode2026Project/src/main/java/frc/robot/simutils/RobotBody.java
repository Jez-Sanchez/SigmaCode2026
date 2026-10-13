package frc.robot.simutils;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.ShapeResult;
import com.github.stephengold.joltjni.StaticCompoundShapeSettings;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EOverrideMassProperties;
import com.github.stephengold.joltjni.readonly.ConstShape;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The robot chassis as one dynamic Jolt body, driven by simple forces.
 *
 * <p>This is the stepping stone before the swerve modules. A force controller pushes the body
 * toward a commanded chassis speed, which is enough to test mass, friction, wall collisions, and
 * the bump. Real wheel contact (raycasts, per-module traction) replaces {@link #applyForces()}
 * later, and the rest of this class stays.
 *
 * <p>Call {@link #applyForces()} once per loop, BEFORE the physics step.
 */
public final class RobotBody {

  /** Physical parameters. Tune these; none are measured values. */
  public record Config(
      double massKg,
      double lengthM, // X, chassis footprint including bumpers
      double widthM, // Y, chassis footprint including bumpers
      double chassisHeightM, // ground-hugging base: frame and bumpers, bottom on the floor
      double
          robotHeightM, // TOTAL height including mechanisms. Above chassisHeightM adds a box on top
      double superLengthM, // X footprint of the part above the chassis
      double superWidthM, // Y footprint of the part above the chassis
      double maxAccelMps2, // limits the push so the sim cannot accelerate impossibly hard
      double maxAngularAccelRadPerSec2,
      double velocityTimeConstantSec, // smaller = snappier tracking of the commanded speed
      float friction) {

    public static Config defaults() {
      return new Config(52.0, 0.9, 0.9, 0.25, 0.6, 0.6, 0.6, 6.0, 20.0, 0.1, 0.1f);
    }
  }

  /** The body origin sits on the floor. The tiny lift keeps it from starting inside the floor. */
  private static final double FLOOR_OFFSET_M = 0.01;

  private final PhysicsWorld world;
  private final Config cfg;
  private final int bodyId;
  private final double momentOfInertiaZ;

  /** Robot-relative commanded speeds: vx forward, vy left, omega counterclockwise. */
  private ChassisSpeeds target = new ChassisSpeeds();

  public RobotBody(
      PhysicsWorld world, Config cfg, double startX, double startY, double startYawRad) {
    this.world = world;
    this.cfg = cfg;
    // Approximation for a uniform rectangular plate, used only to scale the turning torque
    this.momentOfInertiaZ =
        cfg.massKg() * (cfg.lengthM() * cfg.lengthM() + cfg.widthM() * cfg.widthM()) / 12.0;

    ConstShape shape = buildShape(cfg);

    // VERIFY: Quat.sRotation and Vec3.sAxisZ names
    Quat rotation = Quat.sRotation(Vec3.sAxisZ(), (float) startYawRad);

    BodyCreationSettings bcs =
        new BodyCreationSettings(
            shape,
            new RVec3(startX, startY, FLOOR_OFFSET_M),
            rotation,
            EMotionType.Dynamic,
            LayerConstants.ROBOT);
    bcs.setFriction(cfg.friction());
    bcs.setAllowSleeping(false); // a resting robot must still respond to commands

    // Use our mass, and let Jolt calculate the inertia from the shape.
    // VERIFY: these three names against your jolt-jni version.
    bcs.setOverrideMassProperties(EOverrideMassProperties.CalculateInertia);
    bcs.getMassPropertiesOverride().setMass((float) cfg.massKg());

    this.bodyId = world.addBody(bcs, EActivation.Activate);
  }

  // ---------------------------------------------------------------- shape

  /**
   * Compound shape with its origin on the floor at the footprint center:
   *
   * <pre>
   *    superstructure   z = chassisHeight .. robotHeight   (smaller footprint, fills the height)
   *    chassis          z = 0 .. chassisHeight             (full footprint, hugs the ground)
   * </pre>
   *
   * Because the origin is on the floor, the logged Pose3d lines up with a robot model in
   * AdvantageScope, and the trench beam hits the top box exactly at robotHeight.
   */
  private static ConstShape buildShape(Config cfg) {
    StaticCompoundShapeSettings compound = new StaticCompoundShapeSettings();

    // Chassis: a 3 cm convex radius rounds its edges so it can ride up the bump ramp. The swerve
    // model will replace this with real wheels later.
    float chassisH = (float) cfg.chassisHeightM();
    BoxShape chassis =
        new BoxShape(
            new Vec3((float) (cfg.lengthM() / 2.0), (float) (cfg.widthM() / 2.0), chassisH / 2f),
            0.03f);
    compound.addShape(new Vec3(0f, 0f, chassisH / 2f), Quat.sIdentity(), chassis);

    // Superstructure: only if the robot is taller than the chassis
    double superH = cfg.robotHeightM() - cfg.chassisHeightM();
    if (superH > 0.01) {
      float radius = (float) Math.min(0.01, superH / 2.0); // radius cannot exceed the half height
      BoxShape superstructure =
          new BoxShape(
              new Vec3(
                  (float) (cfg.superLengthM() / 2.0),
                  (float) (cfg.superWidthM() / 2.0),
                  (float) (superH / 2.0)),
              radius);
      compound.addShape(
          new Vec3(0f, 0f, (float) (chassisH + superH / 2.0)), Quat.sIdentity(), superstructure);
    }

    // VERIFY: addShape argument order and ShapeResult method names in your jolt-jni version
    ShapeResult result = compound.create();
    if (result.hasError()) {
      throw new IllegalStateException("Robot shape failed: " + result.getError());
    }
    return result.get();
  }

  // ---------------------------------------------------------------- commands

  /** Set the robot-relative speed the chassis should track. Call each loop. */
  public void setTargetSpeeds(ChassisSpeeds robotRelative) {
    this.target = robotRelative;
  }

  /** Same, but given in the field frame. */
  public void setTargetSpeedsFieldRelative(ChassisSpeeds fieldRelative) {
    double yaw = getPose3d().getRotation().getZ();
    this.target =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative, new edu.wpi.first.math.geometry.Rotation2d(yaw));
  }

  // ---------------------------------------------------------------- forces

  /**
   * Push the body toward the commanded speed. Only horizontal force and yaw torque are applied;
   * gravity and contacts handle everything vertical, which is what lets the robot pitch over the
   * bump.
   */
  public void applyForces() {
    BodyInterface bi = world.getBodyInterface();

    // VERIFY: Vec3 accessors may be getX()/getY()/getZ() (floats) or xx()/yy()/zz()
    Vec3 v = bi.getLinearVelocity(bodyId);
    Vec3 w = bi.getAngularVelocity(bodyId);

    double yaw = getPose3d().getRotation().getZ();
    double c = Math.cos(yaw);
    double s = Math.sin(yaw);

    // Commanded velocity, rotated from the robot frame into the world frame
    double targetVx = c * target.vxMetersPerSecond - s * target.vyMetersPerSecond;
    double targetVy = s * target.vxMetersPerSecond + c * target.vyMetersPerSecond;

    double tau = cfg.velocityTimeConstantSec();
    double ax = (targetVx - v.getX()) / tau;
    double ay = (targetVy - v.getY()) / tau;

    // Cap the total acceleration so the sim cannot do the impossible
    double aMag = Math.hypot(ax, ay);
    if (aMag > cfg.maxAccelMps2()) {
      double scale = cfg.maxAccelMps2() / aMag;
      ax *= scale;
      ay *= scale;
    }
    bi.addForce(bodyId, new Vec3((float) (cfg.massKg() * ax), (float) (cfg.massKg() * ay), 0f));

    // Yaw torque toward the commanded rotation rate
    double alpha = (target.omegaRadiansPerSecond - w.getZ()) / tau;
    alpha =
        Math.max(
            -cfg.maxAngularAccelRadPerSec2(), Math.min(cfg.maxAngularAccelRadPerSec2(), alpha));
    bi.addTorque(bodyId, new Vec3(0f, 0f, (float) (momentOfInertiaZ * alpha)));
  }

  // ---------------------------------------------------------------- state

  public Pose3d getPose3d() {
    return Sim3DWorld.toPose3d(world.getBodyInterface(), bodyId);
  }

  public int getBodyId() {
    return bodyId;
  }

  /** Teleport the robot to a 2D pose, level and at rest. Call at the start of auto, for example. */
  public void reset(Pose2d pose) {
    BodyInterface bi = world.getBodyInterface();
    double z = FLOOR_OFFSET_M;

    bi.setPositionAndRotation(
        bodyId,
        new RVec3(pose.getX(), pose.getY(), z),
        Quat.sRotation(Vec3.sAxisZ(), (float) pose.getRotation().getRadians()),
        EActivation.Activate);
    bi.setLinearAndAngularVelocity(bodyId, new Vec3(0f, 0f, 0f), new Vec3(0f, 0f, 0f));
    target = new ChassisSpeeds();
  }
}
