package org.dovershockwave.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[]{};
    public double[] odometryDrivePositionsRad = new double[]{};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveOpenLoop(double output) {}

  default void setTurnOpenLoop(double output) {}

  default void setDriveVelocity(double velocityRadPerSec) {}

  default void setTurnPosition(Rotation2d rotation) {}
}