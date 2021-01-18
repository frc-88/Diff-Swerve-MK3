package frc.robot.swerve;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import frc.robot.swerve.kinematics.InverseKinematics;
import frc.robot.swerve.motion.modifiers.LimitAcceleration;
import frc.robot.swerve.motion.modifiers.ToHammerMode;
import frc.robot.swerve.motion.modifiers.ToRobotCentric;
import frc.robot.swerve.motion.state.MotionState;
import frc.robot.swerve.swervemodule.SwerveModule;
import frc.robot.swerve.swervemodule.SwerveModule.SwitchingMode;
import frc.robot.swerve.util.constants.DoublePreferenceConstant;
import frc.robot.swerve.util.constants.LongPreferenceConstant;
import frc.robot.swerve.wrappers.gyro.Gyro;

/**
 * Represents a complete swerve chassis, with high level operations for
 * controlling it.
 */
public class SwerveChassis {

    // The swerve modules on this chassis.
    private List<SwerveModule> modules;

    // The unmodified commanded target state
    private MotionState targetState;

    // The inverse kinematics controller for this chassis.
    private InverseKinematics inverseKinematics;

    // True if in field-centric mode, false if in robot-centric mode.
    private boolean inFieldCentric = true;
    
    // True if in hammer mode, false otherwise
    private boolean inHammerMode = false;

    // Modifier for field-centric mode
    private ToRobotCentric fieldCentricModifier;

    // Modifier for hammer mode
    private ToHammerMode hammerModeModifier;

    // Modifier for acceleration limiting
    private LimitAcceleration accelerationLimitModifier;

    /**
     * Construct.
     * 
     * @param gyro
     *                    The gyro measuring this chassis's field-oriented angle.
     * @param modules
     *                    The modules on this chassis. Minimum 2.
     * @param expectedUpdateRate The expected rate at which update will be called, in Hz.
     */
    public SwerveChassis(Gyro gyro, double expectedUpdateRate, SwerveModule... modules) {
        Objects.requireNonNull(gyro);

        if (modules.length < 2) {
            throw new IllegalArgumentException("A swerve chassis must have at least 2 swerve modules");
        }
        for (SwerveModule module : modules) {
            Objects.requireNonNull(module);
            module.setSwitchingMode(SwitchingMode.kSmart);
        }
        this.modules = Arrays.asList(modules);

        this.inverseKinematics = new InverseKinematics(modules);

        this.targetState = inverseKinematics.getTargetState();

        fieldCentricModifier = new ToRobotCentric(gyro);
        hammerModeModifier = new ToHammerMode(new DoublePreferenceConstant("Hammer Mode Angle", 70), new LongPreferenceConstant("Hammer Mode Time", 1_000_000));
        accelerationLimitModifier = new LimitAcceleration(new DoublePreferenceConstant("Translation Accel Limit", 100), new DoublePreferenceConstant("Rotation Accel Limit", 1080), inverseKinematics::getTargetState, expectedUpdateRate);
    }

    /**
     * Updates all periodic processes in the swerve chassis, such as setting module
     * controls.
     */
    public void update() {
        MotionState modifiedState = this.targetState;

        // Apply field-centric to translation if necessary
        if (this.inFieldCentric()) {
            modifiedState = fieldCentricModifier.apply(modifiedState);
        }

        // Apply hammer mode if necessary
        if (inHammerMode()) {
            modifiedState = hammerModeModifier.apply(modifiedState);
        }

        // Apply acceleration limits
        modifiedState = accelerationLimitModifier.apply(modifiedState);

        // Set the target state
        this.inverseKinematics.setTargetState(modifiedState);

        // Update the inverse kinematics
        this.inverseKinematics.update();
    }

    /**
     * Sets the target motion state.
     * 
     * @param target The motion state to set
     */
    public void setTargetState(MotionState target) {
        this.targetState = Objects.requireNonNull(target);
    }

    /**
     * Gets the target motion state.
     * 
     * @return The target motion state
     */
    public MotionState getTargetState() {
        return this.targetState;
    }

    /**
     * Sets the max wheel speed.
     * 
     * @param speed
     *                  The maximum wheel speed, in feet per second
     */
    public void setMaxWheelSpeed(double speed) {
        this.inverseKinematics.setMaxSpeed(speed);
    }

    /**
     * Gets the max wheel speed.
     * 
     * @return The maximum wheel speed, in feet per second
     */
    public double getMaxWheelSpeed() {
        return this.inverseKinematics.getMaxSpeed();
    }

    /**
     * Enables field-centric mode, which means that all given translation angles are
     * relative to gyro 0.
     */
    public void setFieldCentic() {
        this.inFieldCentric = true;
    }

    /**
     * Enables robot-centric mode, which means that all given translation angles are
     * relative to the front of the robot.
     */
    public void setRobotCentic() {
        this.inFieldCentric = false;
    }

    /**
     * Is this chassis in field-centric mode?
     * 
     * @return True if in field-centric mode, false if in robot-centric mode
     */
    public boolean inFieldCentric() {
        return this.inFieldCentric;
    }

    /**
     * Enables hammer mode.
     */
    public void enableHammerMode() {
        if (!inHammerMode()) {
            this.inHammerMode = true;
            hammerModeModifier.reset();
            for (SwerveModule module : modules) {
                module.setSwitchingMode(SwitchingMode.kAlwaysSwitch);
            }
        }
    }

    /*
     * Disables hammer mode.
     */
    public void disableHammerMode() {
        if (inHammerMode()) {
            this.inHammerMode = false;
            for (SwerveModule module : modules) {
                module.setSwitchingMode(SwitchingMode.kSmart);
            }
        }
    }

    /**
     * Is this chassis in hammer mode?
     * 
     * @return True if in hammer mode, false otherwise
     */
    public boolean inHammerMode() {
        return this.inHammerMode;
    }
}
