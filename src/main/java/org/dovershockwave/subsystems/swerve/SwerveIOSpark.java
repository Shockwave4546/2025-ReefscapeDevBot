package org.dovershockwave.subsystems.swerve;

import org.dovershockwave.subsystems.swerve.module.Module;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSpark;
import org.dovershockwave.subsystems.swerve.module.ModuleType;

public class SwerveIOSpark implements SwerveIO {
  @Override public org.dovershockwave.subsystems.swerve.module.Module[] getModules() {
    return new Module[] {
      new org.dovershockwave.subsystems.swerve.module.Module(new ModuleIOSpark(ModuleType.FRONT_LEFT), ModuleType.FRONT_LEFT),
      new org.dovershockwave.subsystems.swerve.module.Module(new ModuleIOSpark(ModuleType.FRONT_RIGHT), ModuleType.FRONT_RIGHT),
      new org.dovershockwave.subsystems.swerve.module.Module(new ModuleIOSpark(ModuleType.BACK_LEFT), ModuleType.BACK_LEFT),
      new org.dovershockwave.subsystems.swerve.module.Module(new ModuleIOSpark(ModuleType.BACK_RIGHT), ModuleType.BACK_RIGHT)
    };
  }
}
