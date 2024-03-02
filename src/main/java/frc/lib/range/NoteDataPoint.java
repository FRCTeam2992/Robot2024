package frc.lib.range;

public class NoteDataPoint implements Comparable<NoteDataPoint> {

    // Variables
    private double distance; // Distance calculated by Limelight sighting

    private double mainShooterSpeed;
    private double pivotPosition;

    public NoteDataPoint(double distance, double mainShooterSpeed, double pivotAngle) {
        // Save the Variables
        this.distance = distance;
        this.mainShooterSpeed = mainShooterSpeed;
        this.pivotPosition = pivotAngle;
    }

    public NoteDataPoint() {
        this(0.0, 0.0, 0.0);
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public void setMainShooterSpeed(int speed) {
        this.mainShooterSpeed = speed;
    }

    public double getMainShooterSpeed() {
        return mainShooterSpeed;
    }

    public void setPivotPosition(double position) {
        this.pivotPosition = position;
    }

    public double getPivotPosition() {
        return pivotPosition;
    }

    @Override
    public int compareTo(NoteDataPoint dataPoint) {
        return Double.valueOf(distance).compareTo(dataPoint.getDistance());
    }
}
