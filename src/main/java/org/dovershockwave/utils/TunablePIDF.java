package org.dovershockwave.utils;

import java.util.function.Consumer;

public class TunablePIDF {
  private final TunableBoolean isManualMode;
  private final TunableNumber manualValue;
  private final TunableNumber p;
  private final TunableNumber i;
  private final TunableNumber d;
  private final TunableNumber ff;

  public TunablePIDF(String prefix, PIDFGains defaultGains) {
    isManualMode = new TunableBoolean(prefix + "(1) ManualMode", false);
    manualValue = new TunableNumber(prefix + "(2) ManualValue", 0.0);
    p = new TunableNumber(prefix + "(3) P", defaultGains.p());
    i = new TunableNumber(prefix + "(4) I", defaultGains.i());
    d = new TunableNumber(prefix + "(5) D", defaultGains.d());
    ff = new TunableNumber(prefix + "(6) FF", defaultGains.ff());
  }

  public void periodic(Consumer<PIDFGains> pidfConfigurator, Consumer<Double> manualValueSetter) {
    TunableNumber.ifChanged(hashCode(), values -> pidfConfigurator.accept(new PIDFGains(values[0], values[1], values[2], values[3])), p, i, d, ff);

    if (isManualMode.get()) {
      TunableNumber.ifChanged(hashCode() * 2, values -> manualValueSetter.accept(values[0]), manualValue);
    }
  }

  public boolean isManualMode() {
    return isManualMode.get();
  }
}
