// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hardwareChecks extends SubsystemBase {
  /** Creates a new pigeon2checks. */
  public static Pigeon2 john = new Pigeon2(5, "canivore");
  //private static DigitalInput[] limits = new DigitalInput[2];
  
  public hardwareChecks() {

  }

  public static boolean rollCheck(double a) {
    if (john.getRoll().getValue() <= a) {
    return true; }
    else {
    return false; }
  }

  public static boolean rollCheckGreater(double a) {
    if (john.getRoll().getValue() >= a) {
    return true; }
    else {
    return false; }
  }

  public static boolean rollCheckBetween(double lessthan,double greaterthan) {
    if (john.getRoll().getValue() >= greaterthan && john.getRoll().getValue() <= lessthan) {
      return true;
    }
    return false;
  }

  public static void resets() {
    john.setYaw(0);
  }

  //public static boolean limitCheck(int device) {
    //limits[0] = new DigitalInput(0);
    //limits[1] = new DigitalInput(1);

    //return (limits[device].get());
  //}

  @Override
  public void periodic() {
  }
}
