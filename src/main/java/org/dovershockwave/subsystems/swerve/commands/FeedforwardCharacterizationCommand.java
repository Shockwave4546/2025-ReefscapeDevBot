package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

import java.text.DecimalFormat;
import java.util.LinkedList;

public class FeedforwardCharacterizationCommand extends SequentialCommandGroup {
  private static final double FF_START_DELAY = 2.0;
  private static final double FF_RAMP_RATE = 0.1;

  public FeedforwardCharacterizationCommand(SwerveSubsystem swerve) {
    final var velocitySamples = new LinkedList<Double>();
    final var voltageSamples = new LinkedList<Double>();
    final var timer = new Timer();

    addCommands(
            // Reset data
            new InstantCommand(velocitySamples::clear),
            new InstantCommand(voltageSamples::clear),

            // Allow modules to orient
            new RunCommand(() -> swerve.runDriveCharacterization(0.0), swerve).withTimeout(FF_START_DELAY),

            // Start timer
            new InstantCommand(timer::restart),

            new RunCommand(() -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              swerve.runDriveCharacterization(voltage);
              velocitySamples.add(swerve.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            }, swerve).finallyDo(() -> {
              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              final var formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive FF Characterization Results **********");
              System.out.println("\tkS: " + formatter.format(kS));
              System.out.println("\tkV: " + formatter.format(kV));
            })
    );

    addRequirements(swerve);
  }
}
