package org.dovershockwave.utils;

import java.util.Map;
import java.util.TreeMap;

public class InterpolatingDoubleTreeMap {
  private final TreeMap<Double, Double> map = new TreeMap<>();

  public InterpolatingDoubleTreeMap(Map<Double, Double> data) {
    map.putAll(data);
  }

  public double predict(double key) {
    if (map.containsKey(key)) return map.get(key);

    Double lower = map.floorKey(key);
    Double upper = map.ceilingKey(key);

    if (lower == null) return map.firstEntry().getValue();
    if (upper == null) return map.lastEntry().getValue();

    double lowerValue = map.get(lower);
    double upperValue = map.get(upper);
    return lowerValue + (upperValue - lowerValue) * (key - lower) / (upper - lower);
  }
}