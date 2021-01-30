/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team88.swerve.SwerveChassis;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.swervemodule.motorsensor.CANifiedPWMEncoder;
import frc.team88.swerve.swervemodule.motorsensor.MotorCombiner;
import frc.team88.swerve.swervemodule.motorsensor.PIDFalcon;
import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;
import frc.team88.swerve.swervemodule.motorsensor.PIDTransmission;
import frc.team88.swerve.swervemodule.motorsensor.PositionVelocitySensor;
import frc.team88.swerve.swervemodule.motorsensor.SensorTransmission;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.Constants;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;
import frc.team88.swerve.wrappers.gyro.NavX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private HashMap<String, PIDFalcon> motors;
  private HashMap<String, PIDMotor> outputs;
  private HashMap<String, PositionVelocitySensor> azimuthEncoders;
  private HashMap<String, SwerveModule> modules;
  private NavX navx;
  private SwerveChassis chassis;

  private Joystick gamepad;

  private PIDPreferenceConstants motorSpeedPIDConstants;
  private PIDPreferenceConstants azimuthPositionPIDConstants;

  private static final double AZIMUTH_GEAR_RATIO = 1. / 360.;
  private static final double WHEEL_GEAR_RATIO = 1. / ((1. / 3.5) * Math.PI);

  private static final double WIDTH = 10.991 * 2/ 12.;
  private static final double LENGTH = 12.491 / 12.;

  private static final double MAX_SPEED = 14.7;
  private static final double MAX_ROTATION = 360;

  private boolean calibrateMode = false;

  private WrappedAngle translationAngle = new WrappedAngle(0);

  @Override
  public void robotInit() {
      // Initialize the PID constants
      motorSpeedPIDConstants = new PIDPreferenceConstants("Motor Speed", 0, 0.00000035, 0, 0.00019, 150, 0, 0);
      azimuthPositionPIDConstants = new PIDPreferenceConstants("Azimuth Position", 8.5, 0, 0.15, 0, 0, 0, 0);

      // Create the base NEOs
      motors = new HashMap<>();
      motors.put("fl+", new PIDFalcon(1, motorSpeedPIDConstants));
      motors.put("fl-", new PIDFalcon(16, motorSpeedPIDConstants));
      motors.put("bl+", new PIDFalcon(2, motorSpeedPIDConstants));
      motors.put("bl-", new PIDFalcon(3, motorSpeedPIDConstants));
      motors.put("br+", new PIDFalcon(13, motorSpeedPIDConstants));
      motors.put("br-", new PIDFalcon(12, motorSpeedPIDConstants));
      motors.put("fr+", new PIDFalcon(14, motorSpeedPIDConstants));
      motors.put("fr-", new PIDFalcon(15, motorSpeedPIDConstants));

      // Reversing all neos makes for counterclockise azimuth to be positve
      for (Map.Entry<String, PIDFalcon> entry : motors.entrySet()) {
          entry.getValue().setInverted(true);
      }

      // Create the differential/planetary
      MotorCombiner flCombiner = new MotorCombiner.Builder(2)
              .addInput(motors.get("fl+"), -0.0166667, 0.19444444)
              .addInput(motors.get("fl-"), 0.0166667, 0.22222222).build();
      MotorCombiner blCombiner = new MotorCombiner.Builder(2)
              .addInput(motors.get("bl+"), -0.0166667, 0.077777778)
              .addInput(motors.get("bl-"), 0.0166667, 0.088888889).build();
      MotorCombiner brCombiner = new MotorCombiner.Builder(2)
              .addInput(motors.get("br+"), -0.0166667, 0.077777778)
              .addInput(motors.get("br-"), 0.0166667, 0.088888889).build();
      MotorCombiner frCombiner = new MotorCombiner.Builder(2)
              .addInput(motors.get("fr+"), -0.0166667, 0.077777778)
              .addInput(motors.get("fr-"), 0.0166667, 0.088888889).build();

      // Create the transmissions
      outputs = new HashMap<>();
      outputs.put("FL Azimuth", new PIDTransmission(flCombiner.getOutput(0), AZIMUTH_GEAR_RATIO));
      outputs.put("FL Wheel", new PIDTransmission(flCombiner.getOutput(1), WHEEL_GEAR_RATIO));
      outputs.put("BL Azimuth", new PIDTransmission(blCombiner.getOutput(0), AZIMUTH_GEAR_RATIO));
      outputs.put("BL Wheel", new PIDTransmission(blCombiner.getOutput(1), WHEEL_GEAR_RATIO));
      outputs.put("BR Azimuth", new PIDTransmission(brCombiner.getOutput(0), AZIMUTH_GEAR_RATIO));
      outputs.put("BR Wheel", new PIDTransmission(brCombiner.getOutput(1), WHEEL_GEAR_RATIO));
      outputs.put("FR Azimuth", new PIDTransmission(frCombiner.getOutput(0), AZIMUTH_GEAR_RATIO));
      outputs.put("FR Wheel", new PIDTransmission(frCombiner.getOutput(1), WHEEL_GEAR_RATIO));

      // Create the absolute encoders
      CANifier canifier = new CANifier(21);
      azimuthEncoders = new HashMap<>();
      azimuthEncoders.put("FL",
              new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel0,
                      outputs.get("FL Azimuth")::getVelocity, new DoublePreferenceConstant("FL Az Enc", 0)),
                      AZIMUTH_GEAR_RATIO));
      azimuthEncoders.put("BL",
              new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel1,
                      outputs.get("BL Azimuth")::getVelocity, new DoublePreferenceConstant("BL Az Enc", 0)),
                      AZIMUTH_GEAR_RATIO));
      azimuthEncoders.put("BR",
              new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel2,
                      outputs.get("BR Azimuth")::getVelocity, new DoublePreferenceConstant("BR Az Enc", 0)),
                      AZIMUTH_GEAR_RATIO));
      azimuthEncoders.put("FR",
              new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel3,
                      outputs.get("FR Azimuth")::getVelocity, new DoublePreferenceConstant("FR Az Enc", 0)),
                      AZIMUTH_GEAR_RATIO));
    //   azimuthEncoders.put("FL", outputs.get("FL Azimuth"));
    //   azimuthEncoders.put("FR", outputs.get("FR Azimuth"));
    //   azimuthEncoders.put("BL", outputs.get("BL Azimuth"));
    //   azimuthEncoders.put("BR", outputs.get("BR Azimuth"));

      // Create the modules
      modules = new HashMap<>();
      modules.put("FL", new SwerveModule(outputs.get("FL Wheel"), outputs.get("FL Azimuth"),
              azimuthEncoders.get("FL"), azimuthPositionPIDConstants));
      modules.put("BL", new SwerveModule(outputs.get("BL Wheel"), outputs.get("BL Azimuth"),
              azimuthEncoders.get("BL"), azimuthPositionPIDConstants));
      modules.put("BR", new SwerveModule(outputs.get("BR Wheel"), outputs.get("BR Azimuth"),
              azimuthEncoders.get("BR"), azimuthPositionPIDConstants));
      modules.put("FR", new SwerveModule(outputs.get("FR Wheel"), outputs.get("FR Azimuth"),
              azimuthEncoders.get("FR"), azimuthPositionPIDConstants));

      // Set the module locations
      modules.get("FL").setLocation(Vector2D.createCartesianCoordinates(-WIDTH / 2, LENGTH / 2));
      modules.get("BL").setLocation(Vector2D.createCartesianCoordinates(-WIDTH / 2, -LENGTH / 2));
      modules.get("BR").setLocation(Vector2D.createCartesianCoordinates(WIDTH / 2, -LENGTH / 2));
      modules.get("FR").setLocation(Vector2D.createCartesianCoordinates(WIDTH / 2, LENGTH / 2));

      // Create and zero gyro
      navx = new NavX(Port.kOnboard);
      navx.calibrateYaw(0);

      // Create the chassis
      chassis = new SwerveChassis(navx, 50, modules.get("FL"), modules.get("BL"), modules.get("BR"),
              modules.get("FR"));
      chassis.setMaxWheelSpeed(MAX_SPEED);

      // Create the gamepad
      gamepad = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
      if (gamepad.getRawButton(7)) {
          // OOOOOOOH, I WONDER WHAT THIS DOES!
        //   int infiniteTJSquareds = 88 / 0;
        //   throw new IllegalStateException(
        //           infiniteTJSquareds + " is too much awesome, I don't know how we didn't crash already!");
      }

      Constants.update();
      SmartDashboard.putNumber("Yaw", navx.getYaw());
      SmartDashboard.putNumber("FL Azimuth", azimuthEncoders.get("FL").getPosition());
      SmartDashboard.putNumber("BL Azimuth", azimuthEncoders.get("BL").getPosition());
      SmartDashboard.putNumber("BR Azimuth", azimuthEncoders.get("BR").getPosition());
      SmartDashboard.putNumber("FR Azimuth", azimuthEncoders.get("FR").getPosition());

      SmartDashboard.putNumber("FL Wheel", outputs.get("FL Wheel").getPosition());

      SmartDashboard.putNumber("FL M1 Speed", motors.get("fl-").getVelocity());
      SmartDashboard.putNumber("FL M2 Speed", motors.get("fl+").getVelocity());
      SmartDashboard.putNumber("ratio", motors.get("fl-").getVelocity() / motors.get("fl+").getVelocity());
  }

  @Override
  public void disabledInit() {
      disableCalibrateMode();
  }

  @Override
  public void disabledPeriodic() {
      if (gamepad.getRawButton(7)) {
          enableCalibrateMode();
      }
      if (gamepad.getRawButton(8)) {
          disableCalibrateMode();
      }
      if (gamepad.getRawButton(4)) {
          navx.calibrateYaw(0);
      }
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
      // Check robot-centric mode
      if (gamepad.getRawButton(1)) {
          chassis.setRobotCentic();
      }

      // Check field-centric mode
      if (gamepad.getRawButton(4)) {
          chassis.setFieldCentic();
      }

      // Check hammer mode
      if (gamepad.getRawButton(5)) {
          chassis.enableHammerMode();
      } else {
          chassis.disableHammerMode();
      }

      // The target motion state to set
      MotionState targetState = chassis.getTargetState();

      // Check if the translation angle should be updated
      System.out.println(Math.sqrt(Math.pow(gamepad.getRawAxis(4), 2) + Math.pow(gamepad.getRawAxis(5), 2)));
      if (Math.sqrt(Math.pow(gamepad.getRawAxis(4), 2) + Math.pow(gamepad.getRawAxis(5), 2)) > 0.6) {
          System.out.println(-Math.toDegrees(Math.atan2(gamepad.getRawAxis(4), -gamepad.getRawAxis(5))));
          this.translationAngle = new WrappedAngle(-Math.toDegrees(Math.atan2(gamepad.getRawAxis(4), -gamepad.getRawAxis(5))));
      }

      // Determine the translation speed
      double translationSpeed = gamepad.getRawAxis(3) * (gamepad.getRawAxis(2) * 0.75 + 0.25) * MAX_SPEED;

      // If translation speed is 0, make it slightly larger so the wheels will still
      // turn
      if (translationSpeed == 0.0) {
          translationSpeed = 0.001;
      }

      // Set the translation velocity vector
      targetState = targetState
              .changeTranslationVelocity(Vector2D.createPolarCoordinates(translationSpeed, this.translationAngle));

      // Set the rotation velocity
      if (Math.abs(gamepad.getRawAxis(0)) > 0.15) {
          targetState = targetState.changeRotationVelocity(-Math.signum(gamepad.getRawAxis(0)) * (Math.abs(gamepad.getRawAxis(0)) * 0.9 + 0.1) * MAX_ROTATION);
      } else {
          targetState = targetState.changeRotationVelocity(0);
      }

      // Set the target state
      chassis.setTargetState(targetState);

      // Update the chassis
      chassis.update();
  }

  @Override
  public void testInit() {
      SmartDashboard.putNumber("Set Pos Vel", 0);
      SmartDashboard.putNumber("Set Neg Vel", 0);
      disableCalibrateMode();
  }

  @Override
  public void testPeriodic() {
      if (gamepad.getRawButton(1)) {
          enableCalibrateMode();
      }
      if (gamepad.getRawButton(4)) {
          disableCalibrateMode();
      }

      

      PIDFalcon pos = motors.get("br+");
      PIDFalcon neg = motors.get("br-");

      SmartDashboard.putNumber("Pos Vel", pos.getVelocity());
      SmartDashboard.putNumber("Neg Vel", neg.getVelocity());

      pos.setVelocity(SmartDashboard.getNumber("Set Pos Vel", 0));
      neg.setVelocity(SmartDashboard.getNumber("Set Neg Vel", 0));

      // if (gamepad.getRawButton(3)) {
      // pos.calibratePosition(0);
      // neg.calibratePosition(0);
      // }

      // SmartDashboard.putNumber("+ rot", pos.getPosition());
      // SmartDashboard.putNumber("- rot", neg.getPosition());

      // if (Math.abs(gamepad.getRawAxis(0)) > .15)
      // pos.set(gamepad.getRawAxis(0) * 0.1);
      // else
      // pos.set(0);
      // if (Math.abs(gamepad.getRawAxis(4)) > .15)
      // neg.set(gamepad.getRawAxis(4) * 0.1);
      // else
      // neg.set(0);
  }

  private void enableCalibrateMode() {
      if (!calibrateMode) {
          calibrateMode = true;

          // Set all motors to coast mode
          for (Map.Entry<String, PIDFalcon> motor : motors.entrySet()) {
              motor.getValue().setNeutralMode(NeutralMode.Coast);;
          }
      }
  }

  private void disableCalibrateMode() {
      if (calibrateMode) {
          calibrateMode = false;

          // Set all motors back to brake mode
          for (Map.Entry<String, PIDFalcon> motor : motors.entrySet()) {
              motor.getValue().setNeutralMode(NeutralMode.Brake);
          }
          // Set azimuths to 0
          azimuthEncoders.get("FL").calibratePosition(0);
          azimuthEncoders.get("FR").calibratePosition(0);
          azimuthEncoders.get("BL").calibratePosition(0);
          azimuthEncoders.get("BR").calibratePosition(0);

          // Set gyro to 0
          navx.calibrateYaw(0);
      }
  }
}
