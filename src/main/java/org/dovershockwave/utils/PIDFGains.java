package org.dovershockwave.utils;

public record PIDFGains(double p, double i, double d, double ff) {
  public PIDFGains(double p, double i, double d) {
    this(p, i, d, 0);
  }
}