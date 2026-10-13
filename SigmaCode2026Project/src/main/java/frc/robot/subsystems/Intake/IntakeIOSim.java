package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeRollerMotor;
  private double intakeRollerMotorVoltage = 0.0;

  private final IntakeSimulation intakeSim;

  public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
    intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, Inches.of(26.5), Inches.of(3), IntakeSimulation.IntakeSide.FRONT, 80);
    intakeSim.setGamePiecesCount(0);
    intakeRollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.02, 1),
            DCMotor.getKrakenX60(1).withReduction(1.02));
    Constants.simFuelCount = 50;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRollerDisconnected = false;
    inputs.intakeRollerVoltage = intakeRollerMotorVoltage;
    inputs.intakeRollerCurrent = intakeRollerMotor.getCurrentDrawAmps();
    intakeRollerMotor.update(0.02);
    if (!intakeSim.isRunning()) {
      intakeSim.setGamePiecesCount(Constants.simFuelCount);
    }
  }

  @Override
  public void setIntakeRollerSpeed(double speed) {
    intakeRollerMotorVoltage = 12 * speed;
    intakeRollerMotor.setInputVoltage(intakeRollerMotorVoltage);
    if (speed < 0) {
      intakeSim.startIntake();
      Constants.simFuelCount = intakeSim.getGamePiecesAmount();
    } else {
      intakeSim.stopIntake();
    }
  }

  @Override
  public void setRollerVoltage(double voltage) {
    intakeRollerMotorVoltage = voltage;
    intakeRollerMotor.setInputVoltage(intakeRollerMotorVoltage);
  }
}
