/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.SwerveChassis;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;
import frc.team88.swerve.swervemodule.motorsensor.PIDNeo;
import frc.team88.swerve.swervemodule.motorsensor.PIDTransmission;
import frc.team88.swerve.swervemodule.motorsensor.PositionVelocitySensor;
import frc.team88.swerve.swervemodule.motorsensor.SensorTransmission;
import frc.team88.swerve.swervemodule.motorsensor.CANifiedPWMEncoder;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.Constants;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;
import frc.team88.swerve.wrappers.gyro.NavX;
import frc.team88.swerve.swervemodule.motorsensor.MotorCombiner;

public class Robot extends TimedRobot {

    private HashMap<String, PIDNeo> neos;
    private HashMap<String, PIDMotor> outputs;
    private HashMap<String, PositionVelocitySensor> azimuthEncoders;
    private HashMap<String, SwerveModule> modules;
    private NavX navx;
    private SwerveChassis chassis;

    private CANifier canifier;
    private Joystick gamepad;

    private PIDPreferenceConstants motorSpeedPIDConstants;
    private PIDPreferenceConstants azimuthPositionPIDConstants;

    private static final double AZIMUTH_GEAR_RATIO = 1. / 360.;
    private static final double WHEEL_GEAR_RATIO = 1. / ((1. / 3.) * Math.PI);

    private static final double WIDTH = 24.75 / 12.;
    private static final double LENGTH = 21.75 / 12.;

    private static final double MAX_SPEED = 10;
    private static final double MAX_ROTATION = 360;

    private boolean calibrateMode = false;

    private WrappedAngle translationAngle = new WrappedAngle(0);

    @Override
    public void robotInit() {
        // Initialize the PID constants
        motorSpeedPIDConstants = new PIDPreferenceConstants("Motor Speed", 0, 0.00000035, 0, 0.00019, 150, 0, 0);
        azimuthPositionPIDConstants = new PIDPreferenceConstants("Azimuth Position", 8.5, 0, 0.15, 0, 0, 0, 0);

        // Create the base NEOs
        neos = new HashMap<>();
        neos.put("fl+", new PIDNeo(16, motorSpeedPIDConstants));
        neos.put("fl-", new PIDNeo(1, motorSpeedPIDConstants));
        neos.put("bl+", new PIDNeo(2, motorSpeedPIDConstants));
        neos.put("bl-", new PIDNeo(3, motorSpeedPIDConstants));
        neos.put("br+", new PIDNeo(12, motorSpeedPIDConstants));
        neos.put("br-", new PIDNeo(13, motorSpeedPIDConstants));
        neos.put("fr+", new PIDNeo(14, motorSpeedPIDConstants));
        neos.put("fr-", new PIDNeo(15, motorSpeedPIDConstants));

        // Reversing all neos makes for counterclockise azimuth to be positve
        for (Map.Entry<String, PIDNeo> entry : neos.entrySet()) {
            entry.getValue().setInverted(true);
        }

        // Create the differential/planetary
        MotorCombiner flCombiner = new MotorCombiner.Builder(2)
                .addInput(neos.get("fl+"), 0.0183431952662722, 0.0668091168091168)
                .addInput(neos.get("fl-"), 0.0183431952662722, -0.0535612535612536).build();
        MotorCombiner blCombiner = new MotorCombiner.Builder(2)
                .addInput(neos.get("bl+"), 0.0183431952662722, 0.0668091168091168)
                .addInput(neos.get("bl-"), 0.0183431952662722, -0.0535612535612536).build();
        MotorCombiner brCombiner = new MotorCombiner.Builder(2)
                .addInput(neos.get("br+"), 0.0183431952662722, 0.0668091168091168)
                .addInput(neos.get("br-"), 0.0183431952662722, -0.0535612535612536).build();
        MotorCombiner frCombiner = new MotorCombiner.Builder(2)
                .addInput(neos.get("fr+"), 0.0183431952662722, 0.0668091168091168)
                .addInput(neos.get("fr-"), 0.0183431952662722, -0.0535612535612536).build();

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
        canifier = new CANifier(21);
        azimuthEncoders = new HashMap<>();
        azimuthEncoders.put("FL",
                new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel0,
                        outputs.get("FL Azimuth")::getVelocity, new DoublePreferenceConstant("FL Az Enc", 0)),
                        AZIMUTH_GEAR_RATIO));
        azimuthEncoders.put("BL",
                new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel1,
                        outputs.get("BL Azimuth")::getVelocity, new DoublePreferenceConstant("BL Az Enc", 0)),
                        -AZIMUTH_GEAR_RATIO));
        azimuthEncoders.put("BR",
                new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel2,
                        outputs.get("BR Azimuth")::getVelocity, new DoublePreferenceConstant("BR Az Enc", 0)),
                        AZIMUTH_GEAR_RATIO));
        azimuthEncoders.put("FR",
                new SensorTransmission(new CANifiedPWMEncoder(canifier, PWMChannel.PWMChannel3,
                        outputs.get("FR Azimuth")::getVelocity, new DoublePreferenceConstant("FR Az Enc", 0)),
                        -AZIMUTH_GEAR_RATIO));

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
            int infiniteTJSquareds = 88 / 0;
            throw new IllegalStateException(
                    infiniteTJSquareds + " is too much awesome, I don't know how we didn't crash already!");
        }

        Constants.update();
        SmartDashboard.putNumber("Yaw", navx.getYaw());
        SmartDashboard.putNumber("FL Azimuth", azimuthEncoders.get("FL").getPosition());
        SmartDashboard.putNumber("BL Azimuth", azimuthEncoders.get("BL").getPosition());
        SmartDashboard.putNumber("BR Azimuth", azimuthEncoders.get("BR").getPosition());
        SmartDashboard.putNumber("FR Azimuth", azimuthEncoders.get("FR").getPosition());
    }

    @Override
    public void disabledInit() {
        disableCalibrateMode();
    }

    @Override
    public void disabledPeriodic() {
        if (gamepad.getRawButton(1)) {
            enableCalibrateMode();
        }
        if (gamepad.getRawButton(4)) {
            disableCalibrateMode();
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
        if (gamepad.getMagnitude() > 0.9) {
            this.translationAngle = new WrappedAngle(-gamepad.getDirectionDegrees());
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
        if (Math.abs(gamepad.getRawAxis(4)) > 0.1) {
            targetState = targetState.changeRotationVelocity(-(gamepad.getRawAxis(4) * 0.9 + 0.1) * MAX_ROTATION);
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

        // PIDNeo pos = neos.get("br+");
        // PIDNeo neg = neos.get("br-");

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
            for (Map.Entry<String, PIDNeo> neo : neos.entrySet()) {
                neo.getValue().setIdleMode(IdleMode.kCoast);
            }
        }
    }

    private void disableCalibrateMode() {
        if (calibrateMode) {
            calibrateMode = false;

            // Set all motors back to brake mode
            for (Map.Entry<String, PIDNeo> neo : neos.entrySet()) {
                neo.getValue().setIdleMode(IdleMode.kBrake);
            }
            // Set azimuths to 0
            azimuthEncoders.get("FL").calibratePosition(0);
            azimuthEncoders.get("BL").calibratePosition(0);
            azimuthEncoders.get("BR").calibratePosition(0);
            azimuthEncoders.get("FR").calibratePosition(0);

            // Set gyro to 0
            navx.calibrateYaw(0);
        }
    }
}
