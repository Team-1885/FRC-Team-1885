package us.ilite.common.config;


import us.ilite.common.types.input.ELogitech310;

import java.lang.annotation.ElementType;

public class InputMap {

    /**
     * Driver
     * ----------
     * Turn axis - right joystick x-axis
     * Throttle axis - left joystick y-axis
     * Snail mode - right trigger
     * Limelight lock target - A button
     * Activate climb - start button
     * Mid-Rung climb - L Button
     */
    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS, // TODO Experiment with increased or decreased acceleration rates for snail
        ACTIVATE_CLIMB = ELogitech310.START,
        //MID_RUNG = ELogitech310.L_BTN,
        REFLECTIVE_TAPE_TRACKING = ELogitech310.LEFT_TRIGGER_AXIS,
        CONE_TRACKING = ELogitech310.L_BTN,
        CUBE_TRACKING = ELogitech310.R_BTN,
        OPPONENT_ROBOT_CENTER_TRACKING = ELogitech310.X_BTN, // TODO mark prefers the X button for robot tracking
        OPPONENT_ROBOT_LEFT_TRACKING = ELogitech310.DPAD_LEFT, // TODO if we have left and right tracking to target swerve bot corners
        OPPONENT_ROBOT_RIGHT_TRACKING = ELogitech310.DPAD_RIGHT // TODO we probably wont have a center tracking
                // TODO see if we can have a closest corner button
        //TARGET_LOCK = ELogitech310.LEFT_TRIGGER_AXIS
        ;
    }

    /**
     *
     * Operator
     * ----------
     * Spin intake - X btn
     **/
    public static class OPERATOR {
        public static final ELogitech310
        RETRACT_INTAKE = ELogitech310.RIGHT_TRIGGER_AXIS,
        EXTEND_INTAKE = ELogitech310.LEFT_TRIGGER_AXIS,
        SPIN_ROLLERS = ELogitech310.Y_BTN,
        REVERSE_ROLLERS = ELogitech310.B_BTN,

        // BELOW ARE NOT BEING USED BY INTAKE, SO PROBABLY DELETE/COMMENT OUT ONCE UNNECESSARY STUFF IS DELETED IN TELEOP
        SPIN_FEEDER = ELogitech310.X_BTN,
        REVERSE_FEEDER = ELogitech310.Y_BTN,
        RELEASE_BALLS = ELogitech310.DPAD_LEFT,
        SHOOT_CARGO = ELogitech310.R_BTN,
        STAGE_BALLS = ELogitech310.A_BTN,
        PLACE_CARGO = ELogitech310.L_BTN,

        INCREASE_FEEDER_SPEED = ELogitech310.DPAD_UP,
        DECREASE_FEEDER_SPEED = ELogitech310.DPAD_DOWN;
    }

}
