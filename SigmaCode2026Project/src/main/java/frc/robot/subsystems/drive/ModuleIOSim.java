package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class ModuleIOSim implements ModuleIO {

  private SwerveModuleSimulation moduleSim;

  private final GenericMotorController driveMotor;
  private final GenericMotorController turnMotor;

  private boolean driveCloseLoop = false;
  private boolean turnCloseLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, 0);
  private final PIDController turnController = new PIDController(turnSimP, 0, 0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSim) {

    this.moduleSim = moduleSim;

    this.driveMotor =
        moduleSim
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Current.ofBaseUnits(50, Amps));
    this.turnMotor =
        moduleSim.useGenericControllerForSteer().withCurrentLimit(Current.ofBaseUnits(20, Amps));

    moduleSim.useDriveMotorController(driveMotor);
    moduleSim.useSteerMotorController(turnMotor);

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
    driveMotor.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12, 12)));
    turnMotor.requestVoltage(Volts.of(MathUtil.clamp(turnAppliedVolts, -12, 12)));

    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = moduleSim.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = Math.abs((moduleSim.getDriveMotorStatorCurrent().in(Amps)));

    inputs.turnConnected = true;
    inputs.turnPosition =
        new Rotation2d(MathUtil.angleModulus(moduleSim.getSteerAbsoluteFacing().getRadians()));
    inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = moduleSim.getSteerMotorAppliedVoltage().in(Volts);
    inputs.turnCurrentAmps = Math.abs((moduleSim.getSteerMotorStatorCurrent().in(Amps)));

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
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
