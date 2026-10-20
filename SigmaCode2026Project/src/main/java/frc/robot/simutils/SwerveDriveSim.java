package frc.robot.simutils;

import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Four swerve modules acting on the robot's chassis body.
 *
 * <p>Two ways to drive it:
 *
 * <ul>
 *   <li>{@link #setVoltages}: what your real drive IO will use. Voltages in, encoder-style values
 *       out ({@link #getDrivePositionRad}, {@link #getSteerAngle}, and so on).
 *   <li>{@link #setChassisSpeedsCommand}: a built-in open-loop helper (feedforward drive plus a P
 *       steer loop) so you can test driving before your drive subsystem is wired in.
 * </ul>
 *
 * <p>Call {@link #update} once per physics sub-step, BEFORE stepping the world.
 */
public final class SwerveDriveSim {

  // Steer loop used only by the speed-command helper
  private static final double STEER_KP_VOLTS_PER_RAD = 8.0;

  private final PhysicsWorld world;
  private final int bodyId;
  private final SwerveModuleSim[] modules;
  private final SwerveDriveKinematics kinematics;
  private final double maxWheelSpeedMps;

  private final double[] driveVolts;
  private final double[] steerVolts;

  private ChassisSpeeds commanded = new ChassisSpeeds();
  private boolean useSpeedCommand = false;

  /**
   * @param mounts module positions in the robot frame (x forward, y left), in a consistent order
   */
  public SwerveDriveSim(
      PhysicsWorld world,
      RobotBody robot,
      double robotMassKg,
      Translation2d[] mounts,
      SwerveModuleSim.Config cfg) {
    this.world = world;
    this.bodyId = robot.getBodyId();
    this.modules = new SwerveModuleSim[mounts.length];
    this.driveVolts = new double[mounts.length];
    this.steerVolts = new double[mounts.length];

    for (int i = 0; i < mounts.length; i++) {
      modules[i] = new SwerveModuleSim(world, bodyId, mounts[i], cfg, robotMassKg, mounts.length);
    }
    this.kinematics = new SwerveDriveKinematics(mounts);

    // Free speed at the wheel, used to convert a wheel speed into a feedforward voltage
    double motorFreeSpeed = cfg.driveMotor().freeSpeedRadPerSec;
    this.maxWheelSpeedMps = motorFreeSpeed / cfg.driveGearing() * cfg.wheelRadiusM();
  }

  /** Standard square layout, {@code halfSpan} meters from the center in each direction. */
  public static Translation2d[] squareMounts(double halfSpan) {
    return new Translation2d[] {
      new Translation2d(halfSpan, halfSpan), // front left
      new Translation2d(halfSpan, -halfSpan), // front right
      new Translation2d(-halfSpan, halfSpan), // back left
      new Translation2d(-halfSpan, -halfSpan) // back right
    };
  }

  public SwerveModuleSim getModule(int index) {
    return modules[index];
  }

  // ---------------------------------------------------------------- commands

  /** Direct motor voltages per module. This is what your real IO should call. */
  public void setVoltages(double[] drive, double[] steer) {
    useSpeedCommand = false;
    for (int i = 0; i < modules.length; i++) {
      driveVolts[i] = drive[i];
      steerVolts[i] = steer[i];
    }
  }

  /** Open-loop speed command for testing (robot-relative). Replace with your real drive code. */
  public void setChassisSpeedsCommand(ChassisSpeeds robotRelative) {
    this.commanded = robotRelative;
    this.useSpeedCommand = true;
  }

  private void computeVoltagesFromSpeedCommand() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(commanded);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxWheelSpeedMps);

    for (int i = 0; i < modules.length; i++) {
      double current = modules[i].getSteerAngle().getRadians();
      double targetAngle = states[i].angle.getRadians();
      double speed = states[i].speedMetersPerSecond;

      // Take the short way round: if the wheel must turn more than 90 degrees, flip it and
      // reverse the drive instead.
      double error = MathUtil.angleModulus(targetAngle - current);
      if (Math.abs(error) > Math.PI / 2.0) {
        error = MathUtil.angleModulus(error + Math.PI);
        speed = -speed;
      }

      steerVolts[i] = STEER_KP_VOLTS_PER_RAD * error;
      // Slow the drive while the wheel is still pointing the wrong way
      driveVolts[i] = speed / maxWheelSpeedMps * 12.0 * Math.max(0.0, Math.cos(error));
    }
  }

  // ---------------------------------------------------------------- stepping

  /** Apply all module forces for one sub-step of length dt. */
  public void update(double dt) {
    if (useSpeedCommand) {
      computeVoltagesFromSpeedCommand();
    }

    SwerveModuleSim.BodyState state = readBodyState();
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDriveVoltage(driveVolts[i]);
      modules[i].setSteerVoltage(steerVolts[i]);
      modules[i].update(dt, state);
    }
  }

  private SwerveModuleSim.BodyState readBodyState() {
    BodyInterface bi = world.getBodyInterface();
    Pose3d pose = Sim3DWorld.toPose3d(bi, bodyId);

    // VERIFY: getCenterOfMassPosition name, and the Vec3 accessors (getX vs xx)
    RVec3 com = bi.getCenterOfMassPosition(bodyId);
    Vec3 v = bi.getLinearVelocity(bodyId);
    Vec3 w = bi.getAngularVelocity(bodyId);

    return new SwerveModuleSim.BodyState(
        pose.getTranslation(),
        pose.getRotation(),
        new Translation3d(com.xx(), com.yy(), com.zz()),
        new Translation3d(v.getX(), v.getY(), v.getZ()),
        new Translation3d(w.getX(), w.getY(), w.getZ()));
  }

  // ---------------------------------------------------------------- encoder-style outputs

  public int getModuleCount() {
    return modules.length;
  }

  public double getDrivePositionRad(int module) {
    return modules[module].getDrivePositionRad();
  }

  public double getDriveVelocityRadPerSec(int module) {
    return modules[module].getDriveVelocityRadPerSec();
  }

  public edu.wpi.first.math.geometry.Rotation2d getSteerAngle(int module) {
    return modules[module].getSteerAngle();
  }

  public double getSteerVelocityRadPerSec(int module) {
    return modules[module].getSteerVelocityRadPerSec();
  }

  public double getDriveCurrentAmps(int module) {
    return modules[module].getDriveCurrentAmps();
  }

  // ---------------------------------------------------------------- diagnostics

  public boolean isModuleGrounded(int module) {
    return modules[module].isGrounded();
  }

  public double[] getNormalForces() {
    double[] out = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      out[i] = modules[i].getNormalForceN();
    }
    return out;
  }

  public double getTotalNormalForce() {
    double sum = 0.0;
    for (double f : getNormalForces()) {
      sum += f;
    }
    return sum;
  }
}
