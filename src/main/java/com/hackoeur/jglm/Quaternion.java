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

/**
 *
 * @author Daniel Smith <jellymann@gmail.com>
 */
public class Quaternion
{
    private final float x0, x1, x2, x3; 

    // create a new object with the given components
    public Quaternion(float x0, float x1, float x2, float x3) {
        this.x0 = x0;
        this.x1 = x1;
        this.x2 = x2;
        this.x3 = x3;
    }

    // return a string representation of the invoking object
    @Override
    public String toString() {
        return x0 + " + " + x1 + "i + " + x2 + "j + " + x3 + "k";
    }

    // return the quaternion norm
    public float norm() {
        return (float) FastMath.sqrt(x0*x0 + x1*x1 +x2*x2 + x3*x3);
    }

    // return the quaternion conjugate
    public Quaternion conjugate() {
        return new Quaternion(x0, -x1, -x2, -x3);
    }

    // return a new Quaternion whose value is (this + b)
    public Quaternion add(Quaternion b) {
        Quaternion a = this;
        return new Quaternion(a.x0+b.x0, a.x1+b.x1, a.x2+b.x2, a.x3+b.x3);
    }


    // return a new Quaternion whose value is (this * b)
    public Quaternion multiply(Quaternion b) {
        Quaternion a = this;
        float y0 = a.x0*b.x0 - a.x1*b.x1 - a.x2*b.x2 - a.x3*b.x3;
        float y1 = a.x0*b.x1 + a.x1*b.x0 + a.x2*b.x3 - a.x3*b.x2;
        float y2 = a.x0*b.x2 - a.x1*b.x3 + a.x2*b.x0 + a.x3*b.x1;
        float y3 = a.x0*b.x3 + a.x1*b.x2 - a.x2*b.x1 + a.x3*b.x0;
        return new Quaternion(y0, y1, y2, y3);
    }

    // return a new Quaternion whose value is the inverse of this
    public Quaternion inverse() {
        float d = x0*x0 + x1*x1 + x2*x2 + x3*x3;
        return new Quaternion(x0/d, -x1/d, -x2/d, -x3/d);
    }


    // return a / b
    public Quaternion divide(Quaternion b) {
         Quaternion a = this;
        return a.inverse().multiply(b);
    }

    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof Quaternion)
        {
            Quaternion b = (Quaternion)obj;
            return Precision.equals(x0, b.x0, Compare.QUAT_EPSILON)
                    && Compare.equals(x1, b.x1, Compare.QUAT_EPSILON)
                    && Compare.equals(x2, b.x2, Compare.QUAT_EPSILON)
                    && Compare.equals(x3, b.x3, Compare.QUAT_EPSILON);
        }
        return false;
    }
    
    public boolean equalsWithEpsilon(Object obj, float eps)
    {
        if (obj instanceof Quaternion)
        {
            Quaternion b = (Quaternion)obj;
            return Precision.equals(x0, b.x0, eps)
                    && Compare.equals(x1, b.x1, eps)
                    && Compare.equals(x2, b.x2, eps)
                    && Compare.equals(x3, b.x3, eps);
        }
        return false;
    }

    @Override
    public int hashCode()
    {
        int hash = 5;
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.x0) ^ (Double.doubleToLongBits(this.x0) >>> 32));
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.x1) ^ (Double.doubleToLongBits(this.x1) >>> 32));
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.x2) ^ (Double.doubleToLongBits(this.x2) >>> 32));
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.x3) ^ (Double.doubleToLongBits(this.x3) >>> 32));
        return hash;
    }
    
    public Mat4 toMat4()
    {
        // Converts this quaternion to a rotation matrix.
        //
        // | 1 - 2(y^2 + z^2) 2(xy + wz) 2(xz - wy) 0 |
        // | 2(xy - wz) 1 - 2(x^2 + z^2) 2(yz + wx) 0 |
        // | 2(xz + wy) 2(yz - wx) 1 - 2(x^2 + y^2) 0 |
        // | 0 0 0 1 |

        float x =  x1 + x1;
        float y =  x  + x;
        float z =  x3 + x3;
        float xx = x1 * x;
        float xy = x1 * y;
        float xz = x1 * z;
        float yy = x  * y;
        float yz = x  * z;
        float zz = x3 * z;
        float wx = x0 * x;
        float wy = x0 * y;
        float wz = x0 * z;

        return new Mat4(1 - (yy + zz), xy - wz, xz + wy, 0, xy + wz,
                1 - (xx + zz), yz - wx, 0, xz - wy, yz + wx, 1 - (xx + yy), 0,
                0, 0, 0, 1).transpose();
    }
    
}
