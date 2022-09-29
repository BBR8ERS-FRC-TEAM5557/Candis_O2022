package org.frcteam2910.library.util;

import org.frcteam2910.library.math.Vector2;

/**
 * @param A vector representing translation
 * @param a double representing rotation
 * @param boolean for field oriented or not
 */
public class HolonomicDriveSignal {
    private final Vector2 translation;
    private final double rotation;
    private final boolean fieldOriented;

    public HolonomicDriveSignal(Vector2 translation, double rotation, boolean fieldOriented) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    public Vector2 getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }
}
