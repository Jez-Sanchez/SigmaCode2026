package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import frc.robot.simutils.SwerveModuleSim;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class ModuleIOSim implements ModuleIO {

  private SwerveModuleSimulation moduleSim;
  private boolean driveCloseLoop = false;
  private boolean turnCloseLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, 0);
  private final PIDController turnController = new PIDController(turnSimP, 0, 0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private SwerveModuleSim swerveSim;

  public ModuleIOSim(SwerveModuleSim swerveSim) {

    this.swerveSim= swerveSim;

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveCloseLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSim.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond));

    } else {
      driveController.reset();
    }
    if (turnCloseLoop) {
      turnAppliedVolts = turnController.calculate(moduleSim.getSteerAbsoluteFacing().getRadians());

    } else {
      turnController.reset();
    }
    swerveSim.setDriveVoltage(driveAppliedVolts);
    swerveSim.setSteerVoltage(turnAppliedVolts);

    inputs.driveConnected = true;
    inputs.drivePositionRad = swerveSim.getDrivePositionRad();
    inputs.driveVelocityRadPerSec =  swerveSim.getDriveVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(swerveSim.getDriveCurrentAmps());

    inputs.turnConnected = true;
    inputs.turnPosition = swerveSim.getSteerAngle();
    inputs.turnVelocityRadPerSec = swerveSim.getSteerVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(swerveSim.getSteerCurrentAmps());

    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())//Change later
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();//Change later
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
