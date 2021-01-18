package frc.robot.swerve.motion.modifiers;

import frc.robot.swerve.motion.state.FullVelocityMotionState;
import frc.robot.swerve.motion.state.MotionState;
import frc.robot.swerve.util.Vector2D;
import frc.robot.swerve.util.WrappedAngle;
import frc.robot.swerve.wrappers.gyro.Gyro;

/**
 * Converts a robot-centric motion state to a field-centric motion state using
 * the supplied gyro. Will not modify a field-centric motion state.
 */
public class ToFieldCentric implements MotionModifier<MotionState> {

    // The gyro to use for the conversion
    private Gyro gyro;

    /**
     * Constructor.
     * 
     * @param gyro
     *                 The gyro to use for conversion
     */
    public ToFieldCentric(Gyro gyro) {
        this.gyro = gyro;
    }

    @Override
    public MotionState apply(MotionState state) {
        return state.acceptModifier(this);
    }

    @Override
    public FullVelocityMotionState applyToFullVelocityState(FullVelocityMotionState state) {
        if (state.isFieldCentric()) {
            // Already field-centric, nothing to do
            return state;
        }

        Vector2D currentTranslationVelocity = state.getTranslationVelocity();
        WrappedAngle newAngle = currentTranslationVelocity.getAngle().plus(gyro.getYaw());
        return state.changeTranslationVelocity(currentTranslationVelocity.changeAngle(newAngle)).makeFieldCentric();
    }

}