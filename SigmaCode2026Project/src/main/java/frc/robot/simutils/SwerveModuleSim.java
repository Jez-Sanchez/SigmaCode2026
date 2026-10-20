package frc.robot.simutils;

import static edu.wpi.first.units.Units.Radians;

import com.github.stephengold.joltjni.BodyFilter;
import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.BroadPhaseLayerFilter;
import com.github.stephengold.joltjni.RRayCast;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.RayCastResult;
import com.github.stephengold.joltjni.SpecifiedObjectLayerFilter;
import com.github.stephengold.joltjni.Vec3;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.ArrayDeque;
import java.util.Queue;

/**
 * One swerve module as a raycast wheel on the chassis body.
 *
 * <p>Each sub-step the module: casts a ray down from its axle to find the ground, applies a
 * spring-damper force along the surface normal, projects the wheel's heading onto the contact plane
 * (which is what lets the robot climb a ramp), and applies friction-limited drive and lateral
 * forces at the contact point. The drive and steer motors are WPILib DCMotorSim objects, so the
 * module exposes the same kind of voltage-in, position/velocity-out interface as real hardware.
 *
 * <p>Simplifications: the wheel is a point (a vertical ray), the wheel never visibly slips in its
 * reported encoder (it reports rolling speed), and there is no rolling resistance. All are fixable
 * later without changing this class's interface.
 */
public final class SwerveModuleSim {

  /** Module parameters. Defaults are plausible, not measured. */
  public record Config(
      double wheelRadiusM,
      double driveGearing,
      double steerGearing,
      double steerInertiaKgM2,
      double wheelInertiaKgM2, // drive inertia as seen at the wheel, used when airborne
      double springStiffnessNPerM,
      double springDampingNsPerM,
      double frictionCoefficient,
      double driveCurrentLimitAmps,
      double lateralSlipSpeedMps, // lateral speed at which sideways friction is ~76% of max
      DCMotor driveMotor,
      DCMotor steerMotor) {

    public static Config defaults() {
      return new Config(
          0.0508, // 2 in wheel radius
          6.75, // drive gearing
          150.0 / 7.0, // steer gearing
          0.004,
          0.005,
          40_000.0,
          900.0,
          1.0,
          80.0,
          0.3,
          DCMotor.getKrakenX60(1),
          DCMotor.getKrakenX60(1));
    }
  }

  /** Body state shared by all modules in a sub-step, so it is computed only once. */
  public record BodyState(
      Translation3d origin,
      Rotation3d rotation,
      Translation3d centerOfMass,
      Translation3d linearVelocity,
      Translation3d angularVelocity) {}

  private final PhysicsWorld world;
  private final int bodyId;
  private final Translation2d mount; // robot frame: x forward, y left
  private final Config cfg;
  private final double freeLength; // spring length at which force is zero

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;
  private double driveVolts = 0.0;
  private double steerVolts = 0.0;

  // Filters so the ray only sees the field, never the robot's own body
  private final BroadPhaseLayerFilter broadPhaseFilter = new BroadPhaseLayerFilter();
  private final SpecifiedObjectLayerFilter fieldOnlyFilter =
      new SpecifiedObjectLayerFilter(LayerConstants.FIELD);
  private final BodyFilter bodyFilter = new BodyFilter();

  private static final int CACHE_CAPACITY = 1000;

  private final Queue<Angle> cachedDrivePositions = new ArrayDeque<>();
  private final Queue<Rotation2d> cachedSteerAngles = new ArrayDeque<>();

  private final Angle steerRelativeEncoderOffSet = Radians.of((Math.random() - 0.5) * 30);

  // Last-step results, for logging and tests
  private boolean grounded = false;
  private double normalForceN = 0.0;
  private double longForceN = 0.0;
  private double latForceN = 0.0;
  private double driveCurrentAmps = 0.0;
  private double steerCurrentAmps = 0.0;

  /**
   * @param robotMassKg total mass, used to pick the axle height so the robot rests at z = 0
   * @param moduleCount how many modules share the weight
   */
  public SwerveModuleSim(
      PhysicsWorld world,
      int bodyId,
      Translation2d mount,
      Config cfg,
      double robotMassKg,
      int moduleCount) {
    this.world = world;
    this.bodyId = bodyId;
    this.mount = mount;
    this.cfg = cfg;

    // At rest each spring carries an equal share of the weight and compresses by "sag".
    // Putting the axle at wheelRadius and the free length at radius + sag makes the body origin
    // settle at exactly floor level.
    double sag = (robotMassKg * 9.81 / moduleCount) / cfg.springStiffnessNPerM();
    this.freeLength = cfg.wheelRadiusM() + sag;

    // VERIFY: createDCMotorSystem and DCMotorSim constructor signatures in your WPILib version
    this.driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                cfg.driveMotor(), cfg.wheelInertiaKgM2(), cfg.driveGearing()),
            cfg.driveMotor());
    this.steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                cfg.steerMotor(), cfg.steerInertiaKgM2(), cfg.steerGearing()),
            cfg.steerMotor());
  }

  // ---------------------------------------------------------------- inputs

  public void setDriveVoltage(double volts) {
    this.driveVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  public void setSteerVoltage(double volts) {
    this.steerVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  // ---------------------------------------------------------------- outputs (encoder-style)

  public double getDrivePositionRad() {
    return driveSim.getAngularPositionRad();
  }

  public double getDriveVelocityRadPerSec() {
    return driveSim.getAngularVelocityRadPerSec();
  }

  public Rotation2d getSteerAngle() {
    return new Rotation2d(steerSim.getAngularPositionRad());
  }

  public double getSteerVelocityRadPerSec() {
    return steerSim.getAngularVelocityRadPerSec();
  }

  public double getDriveCurrentAmps() {
    return driveCurrentAmps;
  }

  public double getSteerCurrentAmps() {
    return steerCurrentAmps;
  }

  public boolean isGrounded() {
    return grounded;
  }

  public double getNormalForceN() {
    return normalForceN;
  }

  public Translation2d getMount() {
    return mount;
  }

  public Angle[] getCachedDrivePositionsRad() {
    return cachedDrivePositions.toArray(Angle[]::new);
  }

  public Rotation2d[] getCachedSteerPosition() {
    return cachedSteerAngles.toArray(Rotation2d[]::new);
  }

  // ---------------------------------------------------------------- physics

  /** Run one sub-step: motors, then ground contact, then forces onto the chassis body. */
  public void update(double dt, BodyState s) {
    steerSim.setInputVoltage(steerVolts);
    steerSim.update(dt);

    double r = cfg.wheelRadiusM();

    // Axle position and the body's local "down" direction, in the world frame
    Translation3d axle =
        s.origin().plus(new Translation3d(mount.getX(), mount.getY(), r).rotateBy(s.rotation()));
    Translation3d down = new Translation3d(0.0, 0.0, -1.0).rotateBy(s.rotation());

    // ---- find the ground: a ray from the axle, no longer than the spring's free length
    double rayLength = freeLength;
    RRayCast ray =
        new RRayCast(
            new RVec3(axle.getX(), axle.getY(), axle.getZ()),
            new Vec3(
                (float) (down.getX() * rayLength),
                (float) (down.getY() * rayLength),
                (float) (down.getZ() * rayLength)));
    RayCastResult hit = new RayCastResult();

    // VERIFY: castRay overload and filter classes in your jolt-jni version
    boolean hasHit =
        world
            .getPhysicsSystem()
            .getNarrowPhaseQuery()
            .castRay(ray, hit, broadPhaseFilter, fieldOnlyFilter, bodyFilter);

    if (!hasHit) {
      // Airborne: the wheel spins freely under the motor alone
      grounded = false;
      normalForceN = 0.0;
      longForceN = 0.0;
      latForceN = 0.0;
      driveSim.setInputVoltage(driveVolts);
      driveSim.update(dt);
      driveCurrentAmps = driveSim.getCurrentDrawAmps();
      updateCache();
      return;
    }
    grounded = true;
    steerCurrentAmps = steerSim.getCurrentDrawAmps();

    double distance = hit.getFraction() * rayLength;
    Translation3d contact = axle.plus(down.times(distance));
    Translation3d normal = surfaceNormal(hit, contact, down);

    // ---- normal force: spring plus damper along the contact normal, never pulling
    Translation3d contactVelocity = pointVelocity(s, contact);
    double compression = freeLength - distance;
    double closingSpeed = dot(contactVelocity, normal); // negative when moving into the ground
    double fNormal =
        Math.max(
            0.0,
            cfg.springStiffnessNPerM() * compression - cfg.springDampingNsPerM() * closingSpeed);
    normalForceN = fNormal;

    // ---- wheel direction projected onto the contact plane (this is what lets it climb a ramp)
    double steerAngle = steerSim.getAngularPositionRad();
    Translation3d heading =
        new Translation3d(Math.cos(steerAngle), Math.sin(steerAngle), 0.0).rotateBy(s.rotation());
    Translation3d forward = normalize(heading.minus(normal.times(dot(heading, normal))));
    Translation3d lateral = cross(normal, forward); // points left of the wheel

    double vLong = dot(contactVelocity, forward);
    double vLat = dot(contactVelocity, lateral);

    // ---- drive force: motor model at the rolling speed the ground is imposing on the wheel
    double wheelSpeed = vLong / r; // rad/s
    double current =
        MathUtil.clamp(
            cfg.driveMotor().getCurrent(wheelSpeed * cfg.driveGearing(), driveVolts),
            -cfg.driveCurrentLimitAmps(),
            cfg.driveCurrentLimitAmps());
    double wheelTorque = cfg.driveMotor().getTorque(current) * cfg.driveGearing();
    double fDrive = wheelTorque / r;
    driveCurrentAmps = Math.abs(current);

    // The wheel reports the speed the ground gives it (see class notes about slip)
    driveSim.setState(driveSim.getAngularPositionRad() + wheelSpeed * dt, wheelSpeed);

    // ---- friction circle: drive and lateral forces share one traction budget
    double budget = cfg.frictionCoefficient() * fNormal;
    double fLong = MathUtil.clamp(fDrive, -budget, budget);
    double latBudget = Math.sqrt(Math.max(0.0, budget * budget - fLong * fLong));
    double fLat = -latBudget * Math.tanh(vLat / cfg.lateralSlipSpeedMps());
    longForceN = fLong;
    latForceN = fLat;

    // ---- apply everything at the contact point, so the body gets the right torque for free
    Translation3d force =
        normal.times(fNormal).plus(forward.times(fLong)).plus(lateral.times(fLat));

    // VERIFY: BodyInterface.addForce(bodyId, force, point) overload name
    world
        .getBodyInterface()
        .addForce(
            bodyId,
            new Vec3((float) force.getX(), (float) force.getY(), (float) force.getZ()),
            new RVec3(contact.getX(), contact.getY(), contact.getZ()));
    updateCache();
  }
  /** Cache the current encoder-style positions, retaining only the latest samples. */
  private void updateCache() {
    cachedDrivePositions.add(Radians.of(driveSim.getAngularPositionRad()));
    cachedSteerAngles.add(new Rotation2d(steerSim.getAngularPositionRad()));

    while (cachedDrivePositions.size() > CACHE_CAPACITY) {
      cachedDrivePositions.remove();
    }
    while (cachedSteerAngles.size() > CACHE_CAPACITY) {
      cachedSteerAngles.remove();
    }
  }

  // ---------------------------------------------------------------- geometry helpers

  /** Surface normal at the ray hit, oriented to point away from the ground (toward the robot). */
  private Translation3d surfaceNormal(RayCastResult hit, Translation3d point, Translation3d down) {
    Translation3d normal = down.times(-1.0); // fallback: straight up from the body

    // VERIFY: BodyLockRead usage and getWorldSpaceSurfaceNormal in your jolt-jni version
    BodyLockRead lock =
        new BodyLockRead(world.getPhysicsSystem().getBodyLockInterface(), hit.getBodyId());
    try {
      if (lock.succeeded()) {
        Vec3 n =
            lock.getBody()
                .getWorldSpaceSurfaceNormal(
                    hit.getSubShapeId2(), new RVec3(point.getX(), point.getY(), point.getZ()));
        normal = new Translation3d(n.getX(), n.getY(), n.getZ());
      }
    } finally {
      lock.releaseLock();
    }

    // Make sure it points toward the robot, not into the ground
    if (dot(normal, down) > 0.0) {
      normal = normal.times(-1.0);
    }
    return normalize(normal);
  }

  /** Velocity of a point rigidly attached to the body: v = v_com + w x (p - com). */
  private static Translation3d pointVelocity(BodyState s, Translation3d p) {
    return s.linearVelocity().plus(cross(s.angularVelocity(), p.minus(s.centerOfMass())));
  }

  private static double dot(Translation3d a, Translation3d b) {
    return a.getX() * b.getX() + a.getY() * b.getY() + a.getZ() * b.getZ();
  }

  private static Translation3d cross(Translation3d a, Translation3d b) {
    return new Translation3d(
        a.getY() * b.getZ() - a.getZ() * b.getY(),
        a.getZ() * b.getX() - a.getX() * b.getZ(),
        a.getX() * b.getY() - a.getY() * b.getX());
  }

  private static Translation3d normalize(Translation3d v) {
    double n = v.getNorm();
    return n < 1e-9 ? v : v.times(1.0 / n);
  }
}
