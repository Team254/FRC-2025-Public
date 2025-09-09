package com.team254.lib.subsystems;

public class ServoMotorSubsystemWithFollowersConfig extends ServoMotorSubsystemConfig {
    public static class FollowerConfig {
        public ServoMotorSubsystemConfig config = new ServoMotorSubsystemConfig();
        public boolean inverted = false;
    }

    public FollowerConfig[] followers = new FollowerConfig[] {};
}
