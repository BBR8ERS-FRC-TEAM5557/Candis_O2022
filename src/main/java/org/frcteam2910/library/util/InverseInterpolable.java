package org.frcteam2910.library.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
