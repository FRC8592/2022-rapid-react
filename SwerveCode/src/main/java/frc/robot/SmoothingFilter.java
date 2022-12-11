package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SmoothingFilter {
    ChassisSpeeds[] velocityValues;
    int size;
    int index = 0;

    /**
     * Create smoothing object, will slowly and smoothly adjust speed values until target is hit
     * @param size Size for smoothing array, bigger will be smoothed slower
     */
    public SmoothingFilter(int size) {
        this.size = size;
        velocityValues = new ChassisSpeeds[size];
    }

    /**
     * Take an array of zeros and fill each slot with a speed value until the value is hit
     * @param desiredSpeed Speed to accelerate towards
     */
    public ChassisSpeeds smooth(ChassisSpeeds desiredSpeed, double maxVelocity) {
        double sumXVelocities = 0;
        double sumYVelocities = 0;
        velocityValues[index] = desiredSpeed;
        for(ChassisSpeeds cs: velocityValues) {
            if(!cs.equals(null)) {
                sumXVelocities += cs.vxMetersPerSecond;
                sumYVelocities += cs.vyMetersPerSecond;
            }
        }
        index = (index + 1) % size;
        return new ChassisSpeeds(sumXVelocities / size, sumYVelocities / size, 0);
    }
}