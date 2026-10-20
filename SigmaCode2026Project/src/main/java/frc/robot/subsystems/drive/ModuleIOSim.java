package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simutils.Sim3DWorld;
import frc.robot.simutils.SwerveModuleSim;
import frc.robot.util.SparkUtil;
import java.util.Arrays;

public class ModuleIOSim implements ModuleIO {

  private final Sim3DWorld simWorld;
  private final int moduleIndex;
  private boolean driveCloseLoop = false;
  private boolean turnCloseLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, 0);
  private final PIDController turnController = new PIDController(turnSimP, 0, 0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private SwerveModuleSim swerveSim;

  public ModuleIOSim(Sim3DWorld simWorld, int moduleIndex) {

    this.simWorld = simWorld;
    this.moduleIndex = moduleIndex;

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveCloseLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  simWorld.swerve().getModule(moduleIndex).getDrivePositionRad());

    } else {
      driveController.reset();
    }
    if (turnCloseLoop) {
      turnAppliedVolts =
          turnController.calculate(
              simWorld.swerve().getModule(moduleIndex).getSteerAngle().getRadians());

    } else {
      turnController.reset();
    }
    simWorld.swerve().getModule(moduleIndex).setDriveVoltage(driveAppliedVolts);
    simWorld.swerve().getModule(moduleIndex).setSteerVoltage(turnAppliedVolts);

    inputs.driveConnected = true;
    inputs.drivePositionRad = simWorld.swerve().getModule(moduleIndex).getDrivePositionRad();
    inputs.driveVelocityRadPerSec =
        simWorld.swerve().getModule(moduleIndex).getDriveVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
        Math.abs(simWorld.swerve().getModule(moduleIndex).getDriveCurrentAmps());

    inputs.turnConnected = true;
    inputs.turnPosition = simWorld.swerve().getModule(moduleIndex).getSteerAngle();
    inputs.turnVelocityRadPerSec =
        simWorld.swerve().getModule(moduleIndex).getSteerVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps =
        Math.abs(simWorld.swerve().getModule(moduleIndex).getSteerCurrentAmps());

    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(
                simWorld
                    .swerve()
                    .getModule(moduleIndex)
                    .getCachedDrivePositionsRad()) // Change later
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions =
        simWorld.swerve().getModule(moduleIndex).getCachedSteerPosition(); // Change later
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveCloseLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnCloseLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveCloseLoop = true;
    driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  public void setTurnPos(Rotation2d rotation) {
    turnCloseLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
