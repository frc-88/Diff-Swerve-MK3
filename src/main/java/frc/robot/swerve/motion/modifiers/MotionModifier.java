package frc.robot.swerve.motion.modifiers;

import java.util.function.Function;

import frc.robot.swerve.motion.state.FullVelocityMotionState;
import frc.robot.swerve.motion.state.MotionState;

public interface MotionModifier<R> extends Function<MotionState, R> {

    public R applyToFullVelocityState(FullVelocityMotionState state);

}