package org.dovershockwave.subsystems.swerve.module;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import org.dovershockwave.subsystems.swerve.SparkOdometryThread;
import org.dovershockwave.subsystems.swerve.SwerveConfigs;
import org.dovershockwave.utils.PIDFGains;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static org.dovershockwave.utils.SparkUtils.*;

public class ModuleIOSpark implements ModuleIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase driveSpark;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePID;

  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController turnPID;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turnConnectedDebouncer = new Debouncer(0.5);

  private final ModuleType type;

  public ModuleIOSpark(ModuleType type) {
    driveSpark = new SparkMax(type.driveID, SparkLowLevel.MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    drivePID = driveSpark.getClosedLoopController();

    turnSpark = new SparkMax(type.turnID, SparkLowLevel.MotorType.kBrushless);
    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnPID = turnSpark.getClosedLoopController();

    this.type = type;

    tryUntilOk(driveSpark, 5, spark -> {
      spark.configure(SwerveConfigs.DRIVE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(driveSpark, 5, spark -> {
      spark.getEncoder().setPosition(0.0);
    });

    tryUntilOk(turnSpark, 5, spark -> {
      spark.configure(SwerveConfigs.TURN_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override public void updateInputs(ModuleIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(driveSpark,
            new DoubleSupplier[]{driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
            (values) -> inputs.driveAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.driveConnected = driveConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    HAS_STICKY_FAULT = false;
    useValueIfOk(turnSpark, turnEncoder::getPosition, (value) -> inputs.turnPosition = new Rotation2d(value).minus(Rotation2d.fromRadians(type.angleOffset)), () -> HAS_STICKY_FAULT = true);
    useValueIfOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(turnSpark,
            new DoubleSupplier[]{turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
            (values) -> inputs.turnAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.turnConnected = turnConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream().map((value) -> new Rotation2d(value).minus(Rotation2d.fromRadians(type.angleOffset))).toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override public void setDriveVelocity(double velocityRadPerSec) {
    drivePID.setReference(velocityRadPerSec, SparkBase.ControlType.kVelocity);
  }

  @Override public void setTurnPosition(Rotation2d rotation) {
    final double setpoint = MathUtil.inputModulus(rotation.plus(Rotation2d.fromRadians(type.angleOffset)).getRadians(), 0.0, 2 * Math.PI);
    turnPID.setReference(setpoint, SparkBase.ControlType.kPosition);
  }

  @Override public REVLibError setDrivePIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    return driveSpark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override public REVLibError setTurnPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    return turnSpark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }
}