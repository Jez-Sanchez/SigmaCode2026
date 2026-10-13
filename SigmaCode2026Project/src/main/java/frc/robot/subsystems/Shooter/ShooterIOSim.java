package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

/**
 * Physics-simulation implementation (IOSim) for the Turret subsystem.
 *
 * <p>All three mechanism groups are modelled with WPILib {@link DCMotorSim}:
 *
 * <ul>
 *   <li>Turret rotation (Vortex sim)
 *   <li>Shooter left wheel (Kraken X60 sim)
 *   <li>Shooter right wheel (Kraken X60 sim)
 *   <li>Hood (Neo 550 sim)
 * </ul>
 */
public class ShooterIOSim implements ShooterIO {

  private static final double DT = 0.02; // 50 Hz loop period

  // ── Sim motors ───────────────────────────────────────────────────────────
  private final DCMotorSim turretSim;
  private final DCMotorSim shooterLeftSim;
  private final DCMotorSim shooterRightSim;
  private final DCMotorSim hoodSim;

  // ── Applied voltages ──────────────────────────────────────────────────────
  private double turretAppliedVolts = 0.0;
  private double shooterLeftAppliedVolts = 0.0;
  private double shooterRightAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;

  // ── Closed-loop flags & controllers ──────────────────────────────────────
  private boolean turretClosedLoop = false;
  private boolean shooterClosedLoop = false;
  private boolean hoodClosedLoop = false;

  private final PIDController turretPID = new PIDController(turretSimKp, 0.0, turretSimKd);
  private final PIDController shooterPID = new PIDController(shooterSimKp, 0.0, shooterSimKd);
  private final PIDController hoodPID = new PIDController(hoodSimKp, 0.0, hoodSimKd);

  private double turretSetpointRad = 0.0;
  private double shooterSetpointRadPerSec = 0.0;
  private double hoodSetpointDeg = 0.0;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> fieldSpeeds;

  public ShooterIOSim(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds) {
    this.robotPose = robotPose;
    this.fieldSpeeds = fieldSpeeds;
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretMotorModel, turretMOI, turretGearRatio),
            turretMotorModel);

    shooterLeftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                shooterMotorModel, shooterWheelMOI, shooterWheelGearRatio),
            shooterMotorModel);

    shooterRightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                shooterMotorModel, shooterWheelMOI, shooterWheelGearRatio),
            shooterMotorModel);

    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hoodMotorModel, hoodMOI, hoodGearRatio),
            hoodMotorModel);

    // Turret can wrap around; hood cannot
    turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    // ── Turret closed-loop ────────────────────────────────────────────────
    if (turretClosedLoop) {
      turretAppliedVolts =
          turretPID.calculate(turretSim.getAngularPositionRad(), turretSetpointRad);
    }

    // ── Shooter closed-loop ───────────────────────────────────────────────
    if (shooterClosedLoop) {
      double effort =
          shooterPID.calculate(
              shooterLeftSim.getAngularVelocityRadPerSec(), shooterSetpointRadPerSec);
      shooterLeftAppliedVolts = effort;
      shooterRightAppliedVolts = effort;
    }

    // ── Hood closed-loop ──────────────────────────────────────────────────
    if (hoodClosedLoop) {
      hoodAppliedVolts =
          hoodPID.calculate(Math.toDegrees(hoodSim.getAngularPositionRad()), hoodSetpointDeg);
    }

    // ── Step simulations ──────────────────────────────────────────────────
    turretSim.setInputVoltage(MathUtil.clamp(turretAppliedVolts, -12.0, 12.0));
    shooterLeftSim.setInputVoltage(MathUtil.clamp(shooterLeftAppliedVolts, -12.0, 12.0));
    shooterRightSim.setInputVoltage(MathUtil.clamp(shooterRightAppliedVolts, -12.0, 12.0));
    hoodSim.setInputVoltage(MathUtil.clamp(hoodAppliedVolts, -12.0, 12.0));

    turretSim.update(DT);
    shooterLeftSim.update(DT);
    shooterRightSim.update(DT);
    hoodSim.update(DT);

    // ── Populate inputs ───────────────────────────────────────────────────
    inputs.turretConnected = true;
    inputs.turretPosition = Rotation2d.fromRadians(turretSim.getAngularPositionRad());
    inputs.turretVelocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = turretAppliedVolts;
    inputs.turretCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());

    inputs.shooterLeftConnected = true;
    inputs.shooterRightConnected = true;
    inputs.shooterLeftVelocityRPM = shooterLeftSim.getAngularVelocityRadPerSec() * 60;
    inputs.shooterRightVelocityRPM = shooterRightSim.getAngularVelocityRadPerSec() * 60;
    inputs.shooterLeftAppliedVolts = shooterLeftAppliedVolts;
    inputs.shooterRightAppliedVolts = shooterRightAppliedVolts;
    inputs.shooterLeftCurrentAmps = Math.abs(shooterLeftSim.getCurrentDrawAmps());
    inputs.shooterRightCurrentAmps = Math.abs(shooterRightSim.getCurrentDrawAmps());

    inputs.hoodConnected = true;
    // hoodSim angular position is stored in radians internally; convert to degrees for the API
    inputs.hoodPositionDeg = Math.toDegrees(hoodSim.getAngularPositionRad());
    inputs.hoodVelocityDegPerSec = Math.toDegrees(hoodSim.getAngularVelocityRadPerSec());
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
  }

  // ── Turret Commands ───────────────────────────────────────────────────────

  @Override
  public void setTurretOpenLoop(double outputVolts) {
    turretClosedLoop = false;
    turretPID.reset();
    turretAppliedVolts = outputVolts;
  }

  @Override
  public void setTurretPosition(double angleRad) {
    turretClosedLoop = true;
    turretSetpointRad = angleRad;
  }

  // ── Shooter Wheel Commands ────────────────────────────────────────────────

  @Override
  public void setShooterOpenLoop(double outputVolts) {
    shooterClosedLoop = false;
    shooterPID.reset();
    shooterLeftAppliedVolts = outputVolts;
    shooterRightAppliedVolts = outputVolts;
  }

  @Override
  public void setShooterVelocity(double velocityRotPerMin, boolean fullSpeed) {
    shooterClosedLoop = true;
    shooterSetpointRadPerSec = velocityRotPerMin;
    if (fullSpeed && Constants.simFuelCount > 0) {
      System.out.println("Shooting");
      RebuiltFuelOnFly fuel =
          new RebuiltFuelOnFly(
              robotPose.get().getTranslation(),
              new Translation2d(0, 0.2),
              fieldSpeeds.get(),
              Rotation2d.fromRadians(turretSetpointRad),
              Meters.of(1),
              MetersPerSecond.of(-velocityRotPerMin / 6000 * 20),
              Degrees.of((hoodStartAngle + hoodSetpointDeg + 45) * -1));
      fuel
          // Set the target center to the Rebbuilt Hub of the current alliance
          .withTargetPosition(
              () ->
                  FieldMirroringUtils.toCurrentAllianceTranslation(
                      new Translation3d(0.25, 5.56, 2.3)))
          // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the speaker's
          // "mouth")
          .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
          // Set a callback to run when the fuel hits the target
          .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));
      Constants.simFuelCount--;
      fuel
          // Configure callbacks to visualize the flight trajectory of the projectile
          .withProjectileTrajectoryDisplayCallBack(
          // Callback for when the fuel will eventually hit the target (if configured)
          (pose3ds) ->
              Logger.recordOutput(
                  "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the fuel will eventually miss the target, or if no target is
          // configured
          (pose3ds) ->
              Logger.recordOutput(
                  "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));
      fuel.addGamePieceAfterTouchGround(SimulatedArena.getInstance());
      SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }
  }

  // ── Hood Commands ─────────────────────────────────────────────────────────

  @Override
  public void setHoodOpenLoop(double outputVolts) {
    hoodClosedLoop = false;
    hoodPID.reset();
    hoodAppliedVolts = outputVolts;
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    hoodClosedLoop = true;
    hoodSetpointDeg = MathUtil.clamp(angleDeg, hoodMinDeg, hoodMaxDeg);
  }
}
