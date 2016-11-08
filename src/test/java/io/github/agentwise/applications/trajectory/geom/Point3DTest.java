package io.github.agentwise.applications.trajectory.geom;

import org.junit.Test;

import io.github.agentwise.applications.trajectory.geom.point.Point3D;

import static com.google.common.truth.Truth.assertThat;

/** @author Hoang Tung Dinh */
public class Point3DTest {
  @Test
  public void testDistance() {
    final Point3D p0 = Point3D.origin();
    final Point3D p1 = Point3D.create(1, 2, 3);
    assertThat(Point3D.distance(p0, p1)).isWithin(0.01).of(StrictMath.sqrt(14));
  }
}
