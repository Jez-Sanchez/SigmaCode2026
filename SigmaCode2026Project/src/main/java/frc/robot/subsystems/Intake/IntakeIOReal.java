package frc.robot.subsystems.Intake;

import com.revrobotics.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class IntakeIOReal implements IntakeIO {
  // Instantiates the Speed Controller
  // private TalonFX roller;
  // private TalonFX roller2;
  // old TalonFX Rollers

  private SparkFlex roller;
  private SparkFlex roller2;

  private SparkFlexConfig configIntake = new SparkFlexConfig();

  public IntakeIOReal() {
    // Assigns the Speed Controller to the specified CAN ID and CAN Bus

    // roller = new TalonFX(IntakeConstants.rollerID, new CANBus("PhoenixBus"));
    // roller2 = new TalonFX(IntakeConstants.roller2ID, new CANBus("PhoenixBus"));

    roller = new SparkFlex(IntakeConstants.rollerID, MotorType.kBrushless);
    roller2 = new SparkFlex(IntakeConstants.roller2ID, MotorType.kBrushless);

    // Instantiates and adds the configurations for the Speed Controller
    // TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    configIntake.idleMode(IdleMode.kCoast);
    // Enables and sets max current limit for the speed controller

    // rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.rollerCurrentLimit;
    // rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    configIntake.smartCurrentLimit(40);
    // Sets the speed controller neutral mode
    // rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Applies the speed controller configurations

    // roller.getConfigurator().apply(rollerConfig);
    // roller2.getConfigurator().apply(rollerConfig);

    roller.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    roller.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    inputs.intakeRollerCurrent = roller.getStatorCurrent().getValueAsDouble();
    inputs.intakeRollerDisconnected = !roller.isConnected();

    inputs.intakeRoller2Voltage = roller2.getMotorVoltage().getValueAsDouble();
    inputs.intakeRoller2Current = roller2.getStatorCurrent().getValueAsDouble();
    inputs.intakeRoller2Disconnected = !roller2.isConnected();
  }
//change from kracken terms to rev terms
  @Override
  public void setIntakeRollerSpeed(double speed) {
    roller.set(speed);
    roller2.set(-speed);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    roller.setVoltage(voltage);
    roller2.setVoltage(-voltage);
  }
}
