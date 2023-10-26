package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.hardwareChecks;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        
    }

    public static final class TeleOp {

        //Turtle Constant
        public static double TURTLE_SPEED = 3;

      }

    public static final class Pigeon2stuff {
        public static BooleanSupplier rollCheck5 = () -> hardwareChecks.rollCheck(-12.0);
        public static BooleanSupplier rollCheck0 = () -> hardwareChecks.rollCheckBetween(10.5,-10.5);
        public static BooleanSupplier rollCheck2 = () -> hardwareChecks.rollCheckGreater(12);
    }   

    public static final class DriveConstants {
        public static final String CANIVORE = "canivore";
    }

    public static final class AutoConstants {
        public static final int AutoRunTime = 0;

    }

    public static final class shuffleButtons {

        public static BooleanSupplier turtle = () -> DriveSystem.turtleCurrent();
    }
}
    