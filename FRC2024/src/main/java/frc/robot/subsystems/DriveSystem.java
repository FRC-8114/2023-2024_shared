package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CTRSwerve.*;

public class DriveSystem extends SubsystemBase {


    private static DriveSystem driveSystem = new DriveSystem();

    public static DriveSystem getInstance(){
        return driveSystem;
    }

    String canbusName = "carnivore";

    TalonFX m_driveMotor1 = new TalonFX(11, canbusName);
    TalonFX m_driveMotor2 = new TalonFX(21, canbusName);
    TalonFX m_driveMotor3 = new TalonFX(31, canbusName);
    TalonFX m_driveMotor4 = new TalonFX(41, canbusName);
    
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

    private static boolean turtleToggle = true;
    private Pigeon2 pig = new Pigeon2(5, "canivore");

    SwerveDriveTrainConstants drivetrain =
            new SwerveDriveTrainConstants().withPigeon2Id(5).withCANbusName("canivore").withTurnKp(5);

    Slot0Configs steerGains = new Slot0Configs();
    Slot0Configs driveGains = new Slot0Configs();

    {
        steerGains.kP = 30;
        steerGains.kD = 0.2;
        driveGains.kP = 1;
    }

    SwerveDriveConstantsCreator m_constantsCreator =
            new SwerveDriveConstantsCreator(10, 12.8, 3, 17, steerGains, driveGains);

    /**
     * Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
     * respect to this coordinate system
     *
     * <p>This particular drive base is 22" x 22"
     */

    

    SwerveModuleConstants frontRight =
            m_constantsCreator.createModuleConstants(
                    13, 11, 12, -0.066650390625, Units.inchesToMeters(21.4 / 2.0), Units.inchesToMeters(-21.4 / 2.0));
    SwerveModuleConstants frontLeft =
            m_constantsCreator.createModuleConstants(
                    43, 41, 42, -0.752685546875, Units.inchesToMeters(21.4 / 2.0), Units.inchesToMeters(21.4 / 2.0));
    SwerveModuleConstants backRight =
            m_constantsCreator.createModuleConstants(
                    23, 21, 22, -0.771484375, Units.inchesToMeters(-21.4 / 2.0), Units.inchesToMeters(-21.4 / 2.0));
    SwerveModuleConstants backLeft =
            m_constantsCreator.createModuleConstants(
                    33, 31, 32, -0.96240234375, Units.inchesToMeters(-21.4 / 2.0), Units.inchesToMeters(21.4 / 2.0));

                    
    public CTRSwerveDrivetrain m_drivetrain =
            new CTRSwerveDrivetrain(drivetrain, frontLeft, frontRight, backLeft, backRight);

    XboxController m_joystick = new XboxController(0);

    public Consumer<SwerveModuleState[]> setSwerveModule = states -> {
        m_drivetrain.m_modules[0].apply(states[0]);
        m_drivetrain.m_modules[1].apply(states[1]);
        m_drivetrain.m_modules[2].apply(states[2]);
        m_drivetrain.m_modules[3].apply(states[3]);
    };
    
    public static Rotation2d m_lastTargetAngle = new Rotation2d();

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   m_drivetrain.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 m_drivetrain.PoseSupplier, // Pose supplier
                 m_drivetrain.getKinematics(), // SwerveDriveKinematics
                 new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 setSwerveModule, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }

    public DriveSystem() {
        
    }

    public void roboinit() {
        pig.setYaw(0);
        m_lastTargetAngle = m_drivetrain.getPoseMeters().getRotation();
    }

    public static boolean turtleCurrent() {
        return turtleToggle;
    }

    public void runRemote(ChassisSpeeds hi) {
        m_drivetrain.driveFieldCentric(hi);
    }

    public void runFieldRemote(double xSpeed, double ySpeed, Rotation2d angle) {
        m_drivetrain.driveFullyFieldCentric(xSpeed,ySpeed,angle);
    }

    public void xMe() {
        m_drivetrain.driveStopMotion();
    }

    public void teleopPeriodic() {
        
        double leftY = -m_joystick.getLeftY();
        double leftX = m_joystick.getLeftX();
        double rightX = m_joystick.getRightX();
        double rightY = -m_joystick.getRightY();

        if (Math.abs(leftY) < 0.1 && Math.abs(leftX) < 0.1) {
            leftY = 0;
            leftX = 0;
        }
        if (Math.abs(rightX) < 0.1 && Math.abs(rightY) < 0.1) {
            rightX = 0;
            rightY = 0;
        }
        if (m_joystick.getStartButtonPressed()) {
            if (turtleToggle == false) {
            turtleToggle = true;
            talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            m_driveMotor1.getConfigurator().apply(talonConfigs);
            m_driveMotor2.getConfigurator().apply(talonConfigs);
            m_driveMotor3.getConfigurator().apply(talonConfigs);
            m_driveMotor4.getConfigurator().apply(talonConfigs); }
            else if (turtleToggle == true) {
                turtleToggle = false;
                talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
                m_driveMotor1.getConfigurator().apply(talonConfigs);
                m_driveMotor2.getConfigurator().apply(talonConfigs);
                m_driveMotor3.getConfigurator().apply(talonConfigs);          
                m_driveMotor4.getConfigurator().apply(talonConfigs); }
        } 


        var directions = new ChassisSpeeds();

        if (!turtleToggle) {
        directions.vxMetersPerSecond = leftY * 6;
        directions.vyMetersPerSecond = leftX * -6;
        directions.omegaRadiansPerSecond = rightX * -5;
        }
        else if (turtleToggle) {
        directions.vxMetersPerSecond = leftY * (1 * Constants.TeleOp.TURTLE_SPEED);
        directions.vyMetersPerSecond = leftX * (-1 * Constants.TeleOp.TURTLE_SPEED);
        directions.omegaRadiansPerSecond = rightX * (-5);

        }
        

        /* If we're pressing Y, don't move, otherwise do normal movement */
        if (m_joystick.getLeftBumper()) {
            m_drivetrain.driveStopMotion();
        } else {
            /* If we're fully field centric, we need to be pretty deflected to target an angle */
            if (Math.abs(rightX) > 0.7 || Math.abs(rightY) > 0.7) {
                m_lastTargetAngle = new Rotation2d(rightY, -rightX);
            } else {
                m_lastTargetAngle = new Rotation2d();
            }
            m_drivetrain.driveFieldCentric(directions );
        }

        if (m_joystick.getRightBumper()) {
            m_drivetrain.seedFieldRelative();
            // Make us target forward now to avoid jumps
            m_lastTargetAngle = new Rotation2d();
        }

       
    }
}
