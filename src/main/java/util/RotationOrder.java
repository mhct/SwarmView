package util;

import org.ejml.simple.SimpleMatrix;

/**
 * @author Hoang Tung Dinh
 */
public enum RotationOrder {
    XYZ {
        @Override
        public SimpleMatrix get3dRotationMatrix(
                SimpleMatrix rotationMatrixX, SimpleMatrix rotationMatrixY,
                SimpleMatrix rotationMatrixZ) {
            return rotationMatrixZ.mult(rotationMatrixY).mult(rotationMatrixX);
        }
    },

    ZYX {
        @Override
        public SimpleMatrix get3dRotationMatrix(
                SimpleMatrix rotationMatrixX, SimpleMatrix rotationMatrixY,
                SimpleMatrix rotationMatrixZ) {
            return rotationMatrixX.mult(rotationMatrixY).mult(rotationMatrixZ);
        }
    };

    public abstract SimpleMatrix get3dRotationMatrix(
            SimpleMatrix rotationMatrixX, SimpleMatrix rotationMatrixY,
            SimpleMatrix rotationMatrixZ);

    public RotationOrder getInverseOrder() {
        switch (this) {
        case XYZ:
            return ZYX;
        case ZYX:
            return XYZ;
        default:
            throw new IllegalStateException("The inverse is not defined for the input order.");
        }
    }
}
