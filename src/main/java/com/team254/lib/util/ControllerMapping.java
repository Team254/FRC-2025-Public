package com.team254.lib.util;

import java.util.Map;

public class ControllerMapping {
    private final Map<String, Integer> buttonMap;
    private final Map<String, Integer> axisMap;

    public ControllerMapping(Map<String, Integer> buttonMap, Map<String, Integer> axisMap) {
        this.buttonMap = buttonMap;
        this.axisMap = axisMap;
    }

    public int getButton(String name) {
        return buttonMap.get(name);
    }

    public int getAxis(String name) {
        return axisMap.get(name);
    }
}
