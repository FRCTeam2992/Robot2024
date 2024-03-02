package frc.lib.Ranging;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles creating list of setpoints and distances.
 * <p>
 * This class allows you to create list of setpoints and distances for use with
 * shooters. It can find the closest value for either the setpoint or distance.
 */
public class NoteInterpolator {

    private List<NoteDataPoint> dataPointList;

    public NoteInterpolator() {
        dataPointList = new ArrayList<NoteDataPoint>();
    }

    /**
     * @param distance the distance value.
     * @param setpoint the desired value for at the distance.
     */
    public void addDataPoint(NoteDataPoint dataPoint) {
        dataPointList.add(dataPoint);
        Collections.sort(dataPointList);
    }

    public void removeDataPoint(NoteDataPoint dataPoint) {
        dataPointList.remove(dataPoint);
    }

    /**
     * @param distance the distance value.
     * @return the desired value for at the distance.
     */
    public double calcMainShooterSpeed(double distance) {
        double tempMainSpeed = 0;
        if (dataPointList.size() == 1) {
            tempMainSpeed = dataPointList.get(0).getMainShooterSpeed();
        } else if (dataPointList.size() > 1) {
            NoteDataPoint upperDataPoint = new NoteDataPoint(-1.0, 0.0, 0.0);
            NoteDataPoint lowerDataPoint = new NoteDataPoint(-1.0, 0.0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempMainSpeed = upperDataPoint.getMainShooterSpeed();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempMainSpeed = lowerDataPoint.getMainShooterSpeed();
            } else {
                double upperMainSpeed = upperDataPoint.getMainShooterSpeed();
                double lowerMainSpeed = lowerDataPoint.getMainShooterSpeed();

                tempMainSpeed = lerp(lowerMainSpeed, upperMainSpeed, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        // SmartDashboard.putNumber("Main Shooter Speed", tempMainSpeed);

        return tempMainSpeed;
    }

    public double calcPivotPosition(double distance) {
        double tempHoodPosition = 0;

        if (dataPointList.size() == 1) {
            tempHoodPosition = dataPointList.get(0).getPivotPosition();
        } else if (dataPointList.size() > 1) {
            NoteDataPoint upperDataPoint = new NoteDataPoint(-1.0, 0.0, 0.0);
            NoteDataPoint lowerDataPoint = new NoteDataPoint(-1.0, 0.0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempHoodPosition = upperDataPoint.getPivotPosition();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempHoodPosition = lowerDataPoint.getPivotPosition();
            } else {
                double upperHoodPosition = upperDataPoint.getPivotPosition();
                double lowerHoodPosition = lowerDataPoint.getPivotPosition();

                tempHoodPosition = lerp(lowerHoodPosition, upperHoodPosition, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        // SmartDashboard.putNumber("Hood Position", tempHoodPosition);

        return tempHoodPosition;
    }

    private double lerp(double start, double end, double count) {
        return start + (count * (end - start));
    }
}