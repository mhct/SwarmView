package io.github.agentwise.applications.trajectory;

import com.google.auto.value.AutoValue;

import io.github.agentwise.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;
import io.github.agentwise.util.RotationOrder;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Corkscrew motion around a straight line trajectory defined by an origin and destination point, a
 * radius as perpendicular distance to the straight line (origin-destination) and a frequency to
 * specify the number of revolutions.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public final class CorkscrewTrajectory4D extends PeriodicTrajectory implements FiniteTrajectory4d {

    private static final double EPSILON = 0.00000001d;
    private static final String VELOCITY_ERROR_MESSAGE =
            "enterVelocity component is higher than 1 for the given origin-destination points, "
                    + "enterVelocity, radius and frequency values.";
    private final UnitTrajectory unitTrajectory;
    private final double aroundX;
    private final double aroundY;
    private final Point4D origin;
    private final Point3D destination;

    private Point4DCache cache;

    private CorkscrewTrajectory4D(
            Point4D origin,
            Point3D destination,
            double speed,
            double radius,
            double frequency,
            double phase) {
        super(phase, Point4D.origin(), radius, frequency);
        this.origin = origin;
        this.destination = destination;
        checkArgument(!origin.equals(destination), "Origin should not be the same as destination");
        Point4D destinationProjection = Point4D.from(destination, 0);
        double distance = Point4D.distance(origin, destinationProjection);
        unitTrajectory =
                new UnitTrajectory(
                        CircleTrajectory2D.builder()
                                .setRadius(radius)
                                .setFrequency(frequency)
                                .setPhase(phase)
                                .build(),
                        speed,
                        distance);

        //translate origin to get angles to unit vectors.
        Point4D translated = destinationProjection.minus(origin);

        //find angles to unit trajectory and check components for excessive speeds.
        Point3D speedComponent = Point3D.scale(Point3D.project(translated), speed / distance);

        checkArgument(
                isValidVelocity(speedComponent.getX(), speed, radius, frequency),
                "X " + VELOCITY_ERROR_MESSAGE);
        checkArgument(
                isValidVelocity(speedComponent.getY(), speed, radius, frequency),
                "Y " + VELOCITY_ERROR_MESSAGE);
        checkArgument(
                isValidVelocity(speedComponent.getZ(), speed, radius, frequency),
                "Z " + VELOCITY_ERROR_MESSAGE);

        double y = translated.getY();
        double z = translated.getZ();
        double zyNorm = Math.sqrt(StrictMath.pow(y, 2) + StrictMath.pow(z, 2));

        double arx;
        if (z <= 0) {
            arx = (Math.PI / 2) - (stableAtan(z, y));
            if (y < 0) {
                arx += Math.PI;
            }
        } else {
            arx = ((Math.PI / 2) - (StrictMath.acos(translated.getY() / zyNorm)));
        }

        this.aroundX = arx;

        //          this.aroundX = (Math.PI / 2) - (Math.atan(translated.getZ() / translated.getY
        // ()));

        this.aroundY =
                StrictMath.acos(
                        translated.getX()
                                / StrictMath
                                .sqrt(Math.pow(translated.getX(), 2) + StrictMath.pow(zyNorm, 2)));
        //set initial cache
        this.cache = newCache(Pose.create(0, 0, 0, 0), -1);
    }

    private static double stableAtan(double y, double x) {
        return x == 0 ? Math.signum(y) * Math.PI / 2 : StrictMath.atan(y / x);
    }

    /**
     * Calculates whether for a given enterVelocity vector component, magnitude and perpendicular
     * enterVelocity
     * magnitude from the circle movement parameters, the enterVelocity component is still within
     * acceptable bounds. The enterVelocity component for x, Vx = speedX + circleX where speedX
     * is the x
     * component of the vector with magnitude |v| (specified by speed) and direction specified by
     * destination-origin and where circleX represents the x component of the enterVelocity
     * vector of the
     * circle movement at the moment where this vector attains its highest magnitude in the x
     * component. The circle movement enterVelocity vector is always perpendicular to the speed
     * vector
     * (destination-origin with |v|) and the magnitude of the circle enterVelocity vector depends
     * on the
     * circle movement parameters (radius, frequency).
     *
     * @param speedcomp    the component of the enterVelocity. (eg. Vx as speed*cos(phi) with phi
     *                     the
     *                     angle of
     *                     the vector with regards to the component unit vector.)
     * @param speed        the speed or magnitude of the enterVelocity vector |v|.
     * @param radius       the radius of the circle that attains greatest magnitude perpendicular
     *                     to the
     *                     current component.
     * @param frequencythe frequency of the circle that attains greatest magnitude perpendicular to
     *                     the current component.
     * @return false if for the given arguments, the component enterVelocity is >1.
     */
    private static boolean isValidVelocity(
            double speedcomp, double speed, double radius, double frequency) {
        double rfreq2pi = radius * frequency * TWOPI;
        double vcomp =
                Math.abs(speedcomp) + rfreq2pi * Math.sqrt(1 - Math.pow(speedcomp / speed, 2));
        if (vcomp > MAX_ABSOLUTE_VELOCITY) {
            return false;
        }
        return true;
    }

    static Point4DCache newCache(Pose point, double timeMark) {
        return new AutoValue_CorkscrewTrajectory4D_Point4DCache(point, timeMark);
    }

    static Builder builder() {
        return new Builder();
    }

    private void refreshCache(double time) {
        if (!isEqual(cache.getTimeMark(), time)) {
            setCache(unitTrajectory.getDesiredPosition(time), time);
        }
    }

    private Point4D translationTransform(Point4D toTrans) {
        return rotationTransform(toTrans, aroundX, aroundY).plus(origin);
    }

    private Pose getCachePoint() {
        return this.cache.getDestinationPoint();
    }

    private static boolean isEqual(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    private void setCache(Pose beforeTransPoint, double time) {
        this.cache = newCache(beforeTransPoint, time);
    }

    private static Point4D rotationTransform(Point4D toTrans, double aroundX, double aroundY) {
        return Point4D.from(TrajectoryTransformations.reverseRotation(
                Point3D.project(toTrans), aroundX, aroundY, 0, RotationOrder.XYZ), 0);
    }

    @Override
    public String toString() {
        return "CorkscrewTrajectory4D{"
                + "enterVelocity="
                + unitTrajectory.getSpeed()
                + ", origin point="
                + getOrigin()
                + ", destination point="
                + getDestination()
                + ", radius="
                + unitTrajectory.getRadius()
                + ", frequency="
                + unitTrajectory.getFrequency()
                + '}';
    }

    private Point4D getOrigin() {
        return origin;
    }

    private Point3D getDestination() {
        return destination;
    }

    @Override
    public double getTrajectoryDuration() {
        return unitTrajectory.getTrajectoryDuration();
    }

    @Override
    public Pose getDesiredPosition(double timeInSeconds) {
        refreshCache(timeInSeconds);
        Point4D cachePTransformed = translationTransform(Point4D.from(getCachePoint()));
        return Pose.create(cachePTransformed.getX(),
                cachePTransformed.getY(),
                cachePTransformed.getZ(),
                cachePTransformed.getAngle());
    }

    @AutoValue
    abstract static class Point4DCache {

        public abstract Pose getDestinationPoint();

        public abstract double getTimeMark();
    }

    /**
     * Builder for corkscrew trajectories.
     */
    public static final class Builder {
        private Point4D origin;
        private Point3D destination;
        private double speed;
        private double radius;
        private double frequency;
        private double phase;

        private Builder() {
            origin = Point4D.origin();
            destination = Point3D.origin();
            speed = 1;
            radius = 0.5;
            frequency = 0.3;
        }

        /**
         * Default value is Point4D.origin()
         *
         * @param location The origin of the trajectory.
         * @return this builder
         */
        public Builder setOrigin(Point4D origin) {
            this.origin = origin;
            return this;
        }

        /**
         * Default value is Point3D.origin()
         *
         * @param location The destination of the trajectory.
         * @return this builder
         */
        public Builder setDestination(Point3D destination) {
            this.destination = destination;
            return this;
        }

        /**
         * Default value = 1.
         *
         * @param speed The linear enterVelocity between origin and destination.
         * @return this builder
         */
        public Builder setSpeed(double speed) {
            this.speed = speed;
            return this;
        }

        /**
         * Default radius = 0.5.
         *
         * @param radius The radius of the circle movement.
         * @return this builder
         */
        public Builder setRadius(double radius) {
            this.radius = radius;
            return this;
        }

        /**
         * Default frequency = 0.3.
         *
         * @param frequency The frequency of the circle movement.
         * @return this builder
         */
        public Builder setFrequency(double frequency) {
            this.frequency = frequency;
            return this;
        }

        /**
         * Default value = 0.
         *
         * @param phase the phase displacement of the movement.
         * @return this builder
         */
        public Builder setPhase(double phase) {
            this.phase = phase;
            return this;
        }

        /**
         * @return a new Corkscrew trajectory instance.
         */
        public CorkscrewTrajectory4D build() {
            checkNotNull(this.origin, "You have to Supply an origin with setOrigin()");
            checkNotNull(this.destination, "You have to Supply a destination with setOrigin()");
            return new CorkscrewTrajectory4D(origin, destination, speed, radius, frequency, phase);
        }
    }

    private final class UnitTrajectory implements FiniteTrajectory4d {
        private final LinearTrajectory1D linear;
        private final double endPoint;
        private final double speed;
        private final double frequency;
        private final double radius;
        private final Trajectory2d circlePlane;
        private final Trajectory2d holdAtEnd;
        private final double endOrdinate, endAbscissa;
        private final double duration;

        private UnitTrajectory(CircleTrajectory2D circlePlane, double speed, double endPoint) {
            this.linear = new LinearTrajectory1D(0, speed);
            this.circlePlane = circlePlane;
            this.holdAtEnd = new NoMovement2DTrajectory();
            this.endPoint = endPoint;
            this.speed = speed;
            this.frequency = circlePlane.getFrequency();
            this.radius = circlePlane.getRadius();
            this.duration = endPoint / speed;
            this.endOrdinate = circlePlane.getDesiredPositionOrdinate(endPoint / speed);
            this.endAbscissa = circlePlane.getDesiredPositionAbscissa(endPoint / speed);
        }

        double getSpeed() {
            return speed;
        }

        public double getDesiredPositionX(double timeInSeconds) {
            if (timeInSeconds > duration) {
                return endPoint;
            }
            return linear.getDesiredPosition(timeInSeconds);
        }

        private double getRadius() {
            return radius;
        }

        double getFrequency() {
            return frequency;
        }

        public double getDesiredPositionY(double timeInSeconds) {
            if (timeInSeconds > duration) {
                return holdAtEnd.getDesiredPositionOrdinate(timeInSeconds);
            }
            return circlePlane.getDesiredPositionOrdinate(timeInSeconds);
        }

        public double getDesiredPositionZ(double timeInSeconds) {
            if (timeInSeconds > duration) {
                return holdAtEnd.getDesiredPositionAbscissa(timeInSeconds);
            }
            return circlePlane.getDesiredPositionAbscissa(timeInSeconds);
        }

        public double getDesiredAngleZ(double timeInSeconds) {
            return 0;
        }

        @Override
        public Pose getDesiredPosition(double timeInSeconds) {
            return Pose.create(getDesiredPositionX(timeInSeconds),
                    getDesiredPositionY(timeInSeconds),
                    getDesiredPositionZ(timeInSeconds),
                    getDesiredAngleZ(timeInSeconds));
        }

        @Override
        public double getTrajectoryDuration() {
            return endPoint / speed;
        }

        private class NoMovement2DTrajectory implements Trajectory2d {

            @Override
            public double getDesiredPositionAbscissa(double timeInSeconds) {
                return endAbscissa;
            }

            @Override
            public double getDesiredPositionOrdinate(double timeInSeconds) {
                return endOrdinate;
            }
        }
    }
}
