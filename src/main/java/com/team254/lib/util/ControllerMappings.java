package com.team254.lib.util;

import java.util.HashMap;
import java.util.Map;

public class ControllerMappings {
    public static final ControllerMapping XBOX_MAPPING;
    public static final ControllerMapping DUALSENSE_MAPPING;

    static {
        Map<String, Integer> xboxButtons = new HashMap<>();
        xboxButtons.put("A", 1);
        xboxButtons.put("B", 2);
        xboxButtons.put("X", 4);
        xboxButtons.put("Y", 5);
        xboxButtons.put("LeftBumper", 7);
        xboxButtons.put("RightBumper", 8);
        xboxButtons.put("Back", 11);
        xboxButtons.put("Start", 12);
        xboxButtons.put("LeftStick", 14);
        xboxButtons.put("RightStick", 15);

        Map<String, Integer> xboxAxes = new HashMap<>();
        xboxAxes.put("LeftX", 0);
        xboxAxes.put("LeftY", 1);
        xboxAxes.put("RightX", 2);
        xboxAxes.put("RightY", 3);
        xboxAxes.put("RightTrigger", 4);
        xboxAxes.put("LeftTrigger", 5);

        XBOX_MAPPING = new ControllerMapping(xboxButtons, xboxAxes);

        Map<String, Integer> dualSenseButtons = new HashMap<>();
        dualSenseButtons.put("A", 1);
        dualSenseButtons.put("B", 2);
        dualSenseButtons.put("X", 3);
        dualSenseButtons.put("Y", 4);
        dualSenseButtons.put("LeftBumper", 5);
        dualSenseButtons.put("RightBumper", 6);
        dualSenseButtons.put("Back", 7);
        dualSenseButtons.put("Start", 8);
        dualSenseButtons.put("LeftStick", 10);
        dualSenseButtons.put("RightStick", 11);

        Map<String, Integer> dualSenseAxes = new HashMap<>();
        dualSenseAxes.put("LeftX", 0);
        dualSenseAxes.put("LeftY", 1);
        dualSenseAxes.put("RightX", 4);
        dualSenseAxes.put("RightY", 5);
        dualSenseAxes.put("LeftTrigger", 2);
        dualSenseAxes.put("RightTrigger", 3);

        DUALSENSE_MAPPING = new ControllerMapping(dualSenseButtons, dualSenseAxes);
    }
}
