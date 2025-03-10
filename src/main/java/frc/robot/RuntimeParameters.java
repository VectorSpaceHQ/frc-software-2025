package frc.robot;

public class RuntimeParameters {
    public double y_currentHeight = 0;
    public double y_maxHeight = 0;
    public double y_minHeight = 0;

    public double getCurrentHeight() {
        return y_currentHeight;
    }

    public void setCurrentHeight(double v_currentHeight) {
        y_currentHeight = v_currentHeight;
    }

    public double getMaxHeight() {
        return y_maxHeight;
    }

    public void setMaxHeight(double v_maxHeight) {
        y_maxHeight = v_maxHeight;
    }

    public double getMinHeight() {
        return y_minHeight;
    }

    public void setMinHeight(double v_minHeight) {
        y_minHeight = v_minHeight;
    }
}
