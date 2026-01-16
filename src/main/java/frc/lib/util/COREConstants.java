package frc.lib.util;

import edu.wpi.first.wpilibj.Preferences;

public class COREConstants {
    private String label;
    private constantType type;

    public enum constantType {
        DOUBLE,
        INT,
        STRING
    }

    public COREConstants(String name, double initialValue) {
        label = name;
        Preferences.initDouble(label, initialValue);
        type = constantType.DOUBLE;
    }

    public COREConstants(String name, int initialValue) {
        label = name;
        Preferences.initInt(label, initialValue);
        type = constantType.INT;
    }

    public COREConstants(String name, String initialValue) {
        label = name;
        Preferences.initString(label, initialValue);
        type = constantType.STRING;
    }

    public double get(double defaultValue) {
        // System.out.println("Got default valuer of " + defaultValue);
        return Preferences.getDouble(label, defaultValue);
    }

    public void set(double newValue) {
        Preferences.setDouble(label, newValue);
    }

    public void set(int newValue) {
        Preferences.setInt(label, newValue);
    }

    public void set(String newValue) {
        Preferences.setString(label, newValue);
    }

    public int get(int defaultValue) {
        System.out.println("Got default valuer of " + defaultValue);
        return Preferences.getInt(label, defaultValue);
    }

    public String get(String defaultValue) {
        System.out.println("Got default valuer of " + defaultValue);
        return Preferences.getString(label, defaultValue);
    }

    public String getLabel() {
        return label;
    }

    public String getType() {
        return type.toString();
    }
}