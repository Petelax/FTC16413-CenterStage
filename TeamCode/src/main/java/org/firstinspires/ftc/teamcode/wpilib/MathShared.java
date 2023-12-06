// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.wpilib;

public interface MathShared {
  /**
   * Report an error.
   *
   * @param error the error to set
   * @param stackTrace array of stacktrace elements
   */
  void reportError(String error, StackTraceElement[] stackTrace);

  /**
   * Report usage.
   *
   * @param id the usage id
   * @param count the usage count
   */
  void reportUsage(MathUsageId id, int count);

  /**
   * Get the current time.
   *
   * @return Time in seconds
   */
  double getTimestamp();
}
