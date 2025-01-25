package org.dovershockwave.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.utils.PIDFGains;

public class ModuleIOSim implements ModuleIO {
  private final DCMotor driveGearbox = DCMotor.getNEO(1);
  private final DCMotorSim driveSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, SwerveConstants.DRIVE_MOTOR_REDUCTION),
          driveGearbox
  );
  private final DCMotor turnGearbox = DCMotor.getNeo550(1);
  private final DCMotorSim turnSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, SwerveConstants.TURN_MOTOR_REDUCTION),
          turnGearbox
  );

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;

  private final PIDController drivePID = new PIDController(SwerveConstants.DRIVE_SIM_PIDF.p(), SwerveConstants.DRIVE_SIM_PIDF.i(), SwerveConstants.DRIVE_SIM_PIDF.d());
  private double driveFF = SwerveConstants.DRIVE_SIM_PIDF.ff();
  private final PIDController turnPID = new PIDController(SwerveConstants.TURN_SIM_PIDF.p(), SwerveConstants.TURN_SIM_PIDF.i(), SwerveConstants.TURN_SIM_PIDF.d());
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + drivePID.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      drivePID.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnPID.calculate(turnSim.getAngularPositionRad());
    } else {
      turnPID.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveFF * velocityRadPerSec;
    drivePID.setSetpoint(velocityRadPerSec);
  }

  @Override public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnPID.setSetpoint(rotation.getRadians());
  }

  @Override public void setDrivePIDF(PIDFGains gains) {
    drivePID.setP(gains.p());
    drivePID.setI(gains.i());
    drivePID.setD(gains.d());
    driveFF = gains.ff();
  }

  @Override public void setTurnPIDF(PIDFGains gains) {
    turnPID.setP(gains.p());
    turnPID.setI(gains.i());
    turnPID.setD(gains.d());
  }
}