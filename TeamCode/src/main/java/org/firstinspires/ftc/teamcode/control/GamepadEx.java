package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

import kotlin.Pair;
import kotlin.Triple;

public class GamepadEx {
    ///// Create Gamepad Variables
    // Gamepad States
    public Gamepad currentGamepad = new Gamepad();   // This is to prevent rapid
    public Gamepad previousGamepad = new Gamepad();  // toggling of gamepad inputs.
    // Gamepad Inputs
    public enum GamepadInput {left_stick_x, left_stick_y, right_stick_x, right_stick_y,
                              left_trigger, right_trigger, left_bumper, right_bumper,
                              a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right,
                              start, back}
    public enum InputType {onPress, onButtonHold, onRelease}
    // Gamepad Actions
    public final Map<Pair<GamepadInput, InputType>, Runnable> actions = new HashMap<>();
    public final Map<Triple<GamepadInput, InputType, Boolean>, Runnable> constrainedActions = new HashMap<>();
    /////

    public GamepadEx(Gamepad gamepad) {
        update(gamepad);
    }

    public void update(Gamepad gamepad) {
        // Update gamepad states every loop
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad);

        // Check for gamepad input and execute corresponding actions
        for (Map.Entry<Pair<GamepadInput, InputType>, Runnable> entry : actions.entrySet()) {
            GamepadInput input = entry.getKey().getFirst();
            InputType type = entry.getKey().getSecond();
            Runnable action = entry.getValue();

            if (type == InputType.onPress) {
                if (onPress(input)) {
                    action.run();
                }
            } else if (type == InputType.onButtonHold) {
                if (onButtonHold(input)) {
                    action.run();
                }
                if (trigger(input) != 0) {
                    action.run();
                }
            }
        }

        for (Map.Entry<Triple<GamepadInput, InputType, Boolean>, Runnable> entry : constrainedActions.entrySet()) {
            if (entry.getKey().getThird()) {
                GamepadInput input = entry.getKey().getFirst();
                InputType type = entry.getKey().getSecond();
                Runnable action = entry.getValue();

                if (type == InputType.onPress) {
                    if (onPress(input)) {
                        action.run();
                    }
                } else if (type == InputType.onButtonHold) {
                    if (onButtonHold(input)) {
                        action.run();
                    }
                    if (trigger(input) != 0) {
                        action.run();
                    }
                }
            }
        }
    }

    public void addAction(GamepadInput input, InputType type, Runnable action) {
        Pair<GamepadInput, InputType> pair = new Pair<>(input, type);
        actions.put(pair, action);
    }

    public void addAction(GamepadInput input, InputType type, Boolean constraint, Runnable action) {
        Triple<GamepadInput, InputType, Boolean> triple = new Triple<>(input, type, constraint);
        constrainedActions.put(triple, action);
    }

    public boolean onPress(GamepadInput input) {
        // Check if the input is a button
        if (input.ordinal() >= AdvGamepad.GamepadInput.left_bumper.ordinal() &&
                input.ordinal() <= AdvGamepad.GamepadInput.back.ordinal()) {
            try {
                // Use reflection to get the button value
                boolean currentValue = (boolean) currentGamepad.getClass().getField(input.name()).get(currentGamepad);
                boolean previousValue = (boolean) previousGamepad.getClass().getField(input.name()).get(previousGamepad);
                return currentValue && !previousValue;
            } catch (Exception e) {
                return false; // Handle exceptions (e.g., field not found)
            }
        } else {
            // Handle triggers & joysticks (assume they are not "pressed" like buttons)
            return false;
        }
    }

    public boolean onButtonHold(GamepadInput input) {

        // Check if the input is a button
        if (input.ordinal() >= AdvGamepad.GamepadInput.left_bumper.ordinal() &&
                input.ordinal() <= AdvGamepad.GamepadInput.back.ordinal()) {
            try {
                // Use reflection to get the button value
                boolean currentValue = (boolean) currentGamepad.getClass().getField(input.name()).get(currentGamepad);
                boolean previousValue = (boolean) previousGamepad.getClass().getField(input.name()).get(previousGamepad);
                return currentValue && previousValue;
            } catch (Exception e) {
                return false; // Handle exceptions (e.g., field not found)
            }
        } else {
            // Handle triggers & joysticks (assume they are not "pressed" like buttons)
            return false;
        }
    }

    public double trigger(GamepadInput input) {

        // Check if the input is a trigger
        if (input.ordinal() == AdvGamepad.GamepadInput.left_trigger.ordinal() ||
                input.ordinal() == AdvGamepad.GamepadInput.right_trigger.ordinal()) {
            try {
                // Use reflection to get the trigger value
                return (double) currentGamepad.getClass().getField(input.name()).get(currentGamepad);
            } catch (Exception e) {
                return 0; // Handle exceptions (e.g., field not found)
            }
        } else {
            return 0;
        }
    }
}
