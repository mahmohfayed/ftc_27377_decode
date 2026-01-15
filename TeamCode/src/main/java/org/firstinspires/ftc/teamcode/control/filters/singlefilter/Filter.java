package org.firstinspires.ftc.teamcode.control.filters.singlefilter;

public interface Filter {

    default double calculate(double newValue)  {
        return newValue;
    }

    default void reset() {}
}