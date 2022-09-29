package org.frcteam2910.library.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
