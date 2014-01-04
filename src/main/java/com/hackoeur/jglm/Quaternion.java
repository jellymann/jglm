/*
 * Copyright 2014 Daniel Smith <jellymann@gmail.com>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.hackoeur.jglm;

import com.hackoeur.jglm.support.Compare;
import com.hackoeur.jglm.support.FastMath;
import com.hackoeur.jglm.support.Precision;
import static com.hackoeur.jglm.support.FastMath.*;

/**
 *
 * @author Daniel Smith <jellymann@gmail.com>
 */
public class Quaternion
{

    public static Quaternion QUAT_IDENT = new Quaternion(1, 0, 0, 0);
    public static float QUAT_DELTA = 0.001f;

    private final float w, x, y, z;
    
    /**
     * Creates a Quaternion from a axis and a angle.
     * 
     * @param axis
     *            axis vector (will be normalized)
     * @param angle
     *            angle in radians.
     * 
     * @return new quaternion
     */
    public static Quaternion createFromAxisAngle(Vec3 axis, float angle) {
        angle *= 0.5;
        float sin = (float) sin(angle);
        float cos = (float) cos(angle);
        Quaternion q = new Quaternion(cos, axis.getNormalizedTo(sin));
        return q;
    }
    
    /**
     * Creates a Quaternion from Euler angles.
     * 
     * @param pitch
     *            X-angle in radians.
     * @param yaw
     *            Y-angle in radians.
     * @param roll
     *            Z-angle in radians.
     * 
     * @return new quaternion
     */
    public static Quaternion createFromEuler(float pitch, float yaw, float roll) {
        pitch *= 0.5;
        yaw *= 0.5;
        roll *= 0.5;
        double sinPitch = sin(pitch);
        double cosPitch = cos(pitch);
        double sinYaw = sin(yaw);
        double cosYaw = cos(yaw);
        double sinRoll = sin(roll);
        double cosRoll = cos(roll);
        double cosPitchCosYaw = cosPitch * cosYaw;
        double sinPitchSinYaw = sinPitch * sinYaw;

        return new Quaternion(
                (float) (cosRoll * cosPitchCosYaw + sinRoll * sinPitchSinYaw),
                (float) (sinRoll * cosPitchCosYaw - cosRoll * sinPitchSinYaw),
                (float) (cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw),
                (float) (cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw)
        );
    }

    // create a new object with the given components
    public Quaternion(float w, float x, float y, float z)
    {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Quaternion(float w, Vec3 v) {
        this.x = v.getX();
        this.y = v.getY();
        this.z = v.getZ();
        this.w = w;
    }


    // return a string representation of the invoking object
    @Override
    public String toString()
    {
        return w + " + " + x + "i + " + y + "j + " + z + "k";
    }

    // return the quaternion norm
    public float norm()
    {
        return (float) FastMath.sqrt(w * w + x * x + y * y + z * z);
    }

    // return the quaternion conjugate
    public Quaternion conjugate()
    {
        return new Quaternion(w, -x, -y, -z);
    }

    // return a new Quaternion whose value is (this + b)
    public Quaternion add(Quaternion b)
    {
        Quaternion a = this;
        return new Quaternion(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z);
    }

    // return a new Quaternion whose value is (this * b)
    public Quaternion multiply(Quaternion b)
    {
        Quaternion a = this;
        float y0 = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        float y1 = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        float y2 = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        float y3 = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
        return new Quaternion(y0, y1, y2, y3);
    }

    public Quaternion scale(float s)
    {
        return new Quaternion(w * s, x * s, y * s, z * s);
    }

    // return a new Quaternion whose value is the inverse of this
    public Quaternion inverse()
    {
        float d = w * w + x * x + y * y + z * z;
        return new Quaternion(w / d, -x / d, -y / d, -z / d);
    }

    // return a / b
    public Quaternion divide(Quaternion b)
    {
        Quaternion a = this;
        return a.inverse().multiply(b);
    }

    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof Quaternion)
        {
            Quaternion b = (Quaternion) obj;
            return Precision.equals(w, b.w, Compare.QUAT_EPSILON)
                    && Compare.equals(x, b.x, Compare.QUAT_EPSILON)
                    && Compare.equals(y, b.y, Compare.QUAT_EPSILON)
                    && Compare.equals(z, b.z, Compare.QUAT_EPSILON);
        }
        return false;
    }

    public boolean equalsWithEpsilon(Object obj, float eps)
    {
        if (obj instanceof Quaternion)
        {
            Quaternion b = (Quaternion) obj;
            return Precision.equals(w, b.w, eps)
                    && Compare.equals(x, b.x, eps)
                    && Compare.equals(y, b.y, eps)
                    && Compare.equals(z, b.z, eps);
        }
        return false;
    }

    @Override
    public int hashCode()
    {
        int hash = 5;
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.w) ^ (Double.doubleToLongBits(this.w) >>> 32));
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.x) ^ (Double.doubleToLongBits(this.x) >>> 32));
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.y) ^ (Double.doubleToLongBits(this.y) >>> 32));
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.z) ^ (Double.doubleToLongBits(this.z) >>> 32));
        return hash;
    }

    public Mat4 toMat4()
    {
        // Converts this quaternion to a rotation matrix.
        //
        // | 1 - 2(y^2 + z^2) 2(xy + wz)       2(xz - wy)       0 |
        // | 2(xy - wz)       1 - 2(x^2 + z^2) 2(yz + wx)       0 |
        // | 2(xz + wy)       2(yz - wx)       1 - 2(x^2 + y^2) 0 |
        // | 0                0                0                1 |

        float x2 =  x + x;
        float y2 =  y + y;
        float z2 =  z + z;
        float xx =  x * x2;
        float xy =  x * y2;
        float xz =  x * z2;
        float yy =  y * y2;
        float yz =  y * z2;
        float zz =  z * z2;
        float wx =  w * x2;
        float wy =  w * y2;
        float wz =  w * z2;

        return new Mat4(
                1 - (yy + zz), xy - wz,       xz + wy,       0,
                xy + wz,       1 - (xx + zz), yz - wx,       0,
                xz - wy,       yz + wx,       1 - (xx + yy), 0,
                0,             0,             0,             1
        ).transpose();
    }

    public Quaternion slerp(Quaternion to, float t)
    {
        float[] to1 = new float[4];
        double omega, cosom, sinom, scale0, scale1;
        // calc cosine
        cosom = x * to.x + y * to.y + z * to.z
                + w * to.w;
        // adjust signs (if necessary)
        if (cosom < 0.0)
        {
            cosom = -cosom;
            to1[0] = -to.x;
            to1[1] = -to.y;
            to1[2] = -to.z;
            to1[3] = -to.w;
        } else
        {
            to1[0] = to.x;
            to1[1] = to.y;
            to1[2] = to.z;
            to1[3] = to.w;
        }
        // calculate coefficients
        if ((1.0 - cosom) > QUAT_DELTA)
        {
            // standard case (slerp)
            omega = acos(cosom);
            sinom = sin(omega);
            scale0 = sin((1.0 - t) * omega) / sinom;
            scale1 = sin(t * omega) / sinom;
        } else
        {
        // "from" and "to" quaternions are very close 
            //  ... so we can do a linear interpolation
            scale0 = 1.0 - t;
            scale1 = t;
        }
        // calculate final values
        return new Quaternion(
                (float) (scale0 * w + scale1 * to1[3]),
                (float) (scale0 * x + scale1 * to1[0]),
                (float) (scale0 * y + scale1 * to1[1]),
                (float) (scale0 * z + scale1 * to1[2])
        );
    }

}
