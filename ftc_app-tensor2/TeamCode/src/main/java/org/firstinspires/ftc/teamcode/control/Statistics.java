package org.firstinspires.ftc.teamcode.control;

public class Statistics {

    private int n;
    private double sum;
    private double sumsq;

    public void reset() {
        this.n = 0;
        this.sum = 0.0;
        this.sumsq = 0.0;
    }

    public synchronized void addValue(double x) {
        ++this.n;
        this.sum += x;
        this.sumsq += x*x;
    }

    public synchronized double calculateMean() {
        double mean = 0.0;
        if (this.n > 0) {
            mean = this.sum/this.n;
        }
        return mean;
    }

    public synchronized double calculateVariance() {
        double deviation = calculateStandardDeviation();
        return deviation*deviation;
    }

    public synchronized double calculateStandardDeviation() {
        double deviation = 0.0;
        if (this.n > 1) {
            deviation = Math.sqrt((this.sumsq - this.sum*this.sum/this.n)/(this.n-1));
        }
        return deviation;
    }
}
