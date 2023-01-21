package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad

enum class InputType {
    DIGITAL,
    ANALOG,
}

enum class InputMode {
    TOGGLE,
    PUSH,
    LINEAR
}

// TODO ... SOMETHING
var toggleGrabberState = false

// TODO BUILD ACTION MAP OF INPUTS
enum class ActionType(val inputType: InputType, val inputMode: InputMode, val action: (InputMethod, Gamepad) -> Unit) {
    GRABBER_ROTATION(InputType.ANALOG, InputMode.LINEAR, {im, gp ->
        Hardware.rotateArmServo.position = im.getValue!!(gp).toDouble()
    }),
    GRABBER_CLAW(InputType.ANALOG, InputMode.LINEAR, {im, gp ->
        toggleGrabberState = false
        Hardware.grabberServo.position = im.getValue!!(gp).toDouble()
    }),
    GRABBER_TOGGLE(InputType.DIGITAL, InputMode.TOGGLE, {im, gp ->
        if(im.isPressed!!(gp)) {
            toggleGrabberState = !toggleGrabberState
            if(toggleGrabberState)
                Hardware.grabberServo.position = 1.0
            else
                Hardware.grabberServo.position = 0.0
        }
    }),
    X_AXIS(InputType.ANALOG, InputMode.LINEAR, {im, gp ->
        // TODO MOVEMENT
    }),
    Y_AXIS(InputType.ANALOG, InputMode.LINEAR, {im, gp ->
        // TODO MOVEMENT
    }),
    LIFT(InputType.ANALOG, InputMode.LINEAR, {im, gp ->
        // TODO LIFT
    }),
    CLUTCH(InputType.DIGITAL, InputMode.PUSH, {im, gp ->
        // TODO CLUTCH
    }),
    SNAP_LEFT(InputType.DIGITAL, InputMode.PUSH, {im, gp ->
        // TODO SNAP TURNING
    }),
    SNAP_RIGHT(InputType.DIGITAL, InputMode.PUSH, {im, gp ->
        // TODO SNAP RIGHT
    });

    init {
        if(inputType == InputType.DIGITAL && inputMode == InputMode.LINEAR) {
            throw IllegalArgumentException("Digital Input cannot have Linear mode")
        }
    }
}

enum class InputMethod(
    val inputType: InputType,
    val isPressed: ((Gamepad) -> Boolean)? = null,
    val getValue: ((Gamepad) -> Float)? = null) {

    LEFT_TRIGGER(InputType.ANALOG, getValue = { it.left_trigger }),
    LEFT_TRIGGER_DOWN(InputType.DIGITAL, isPressed = { it.left_trigger > 0.5 }),
    RIGHT_TRIGGER(InputType.ANALOG, getValue = { it.right_trigger }),
    RIGHT_TRIGGER_DOWN(InputType.ANALOG, isPressed = { it.right_trigger > 0.5 }),
    LEFT_BUMPER(InputType.DIGITAL, isPressed = { it.left_bumper }),
    RIGHT_BUMPER(InputType.DIGITAL, isPressed = { it.right_bumper }),
    LEFT_STICK_X(InputType.ANALOG, getValue = { it.left_stick_x }),
    LEFT_STICK_Y(InputType.ANALOG, getValue = { it.left_stick_y }),
    LEFT_STICK_PRESS(InputType.DIGITAL, isPressed = { it.left_stick_button }),
    RIGHT_STICK_X(InputType.ANALOG, getValue = { it.right_stick_x }),
    RIGHT_STICK_Y(InputType.ANALOG, getValue = { it.right_stick_y }),
    RIGHT_STICK_PRESS(InputType.DIGITAL, isPressed = { it.right_stick_button }),
    A(InputType.DIGITAL, isPressed = { it.a }),
    B(InputType.DIGITAL, isPressed = { it.b }),
    X(InputType.DIGITAL, isPressed = { it.x }),
    Y(InputType.DIGITAL, isPressed = { it.y });

    init {
        if(isPressed == null  && getValue == null) {
            throw IllegalArgumentException("isPressed and getValue are null")
        } else if (inputType == InputType.DIGITAL && isPressed == null) {
            throw IllegalArgumentException("isPressed is null on digital input")
        } else if (inputType == InputType.ANALOG && getValue == null) {
            throw IllegalArgumentException("getValue is null on analog input")
        }
    }
}

class ActionMap(val mappings: MutableMap<Gamepad, MutableMap<InputMethod, ActionType>> = mutableMapOf()) {
    fun register(gp: Gamepad, im: InputMethod, at: ActionType) {
        if(!mappings.containsKey(gp))
            mappings[gp] = mutableMapOf()
        mappings[gp]?.set(im, at)
    }
    // TODO ADD POLLING AND ACTION DISPATCH
}