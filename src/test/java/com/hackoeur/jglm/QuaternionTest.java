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
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author Daniel Smith <jellymann@gmail.com>
 */
public class QuaternionTest
{
    
    public QuaternionTest()
    {
    }
    
    @BeforeClass
    public static void setUpClass()
    {
    }
    
    @AfterClass
    public static void tearDownClass()
    {
    }
    
    @Before
    public void setUp()
    {
    }
    
    @After
    public void tearDown()
    {
    }

    /**
     * Test of toString method, of class Quaternion.
     */
    @Test
    public void testToString()
    {
        System.out.println("toString");
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        String expResult = "3.0 + 1.0i + 0.0j + 0.0k";
        String result = instance.toString();
        assertEquals(expResult, result);
    }

    /**
     * Test of norm method, of class Quaternion.
     */
    @Test
    public void testNorm()
    {
        System.out.println("norm");
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        float expResult = 3.1622776601683795f;
        float result = instance.norm();
        assertEquals(expResult, result, Compare.QUAT_EPSILON);
    }

    /**
     * Test of conjugate method, of class Quaternion.
     */
    @Test
    public void testConjugate()
    {
        System.out.println("conjugate");
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        Quaternion expResult = new Quaternion(3.0f, -1.0f, -0.0f, -0.0f);
        Quaternion result = instance.conjugate();
        assertEquals(expResult, result);
    }

    /**
     * Test of add method, of class Quaternion.
     */
    @Test
    public void testAdd()
    {
        System.out.println("add");
        Quaternion b = new Quaternion(0.0f, 5.0f, 1.0f, -2.0f);
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        Quaternion expResult = new Quaternion(3.0f, 6.0f, 1.0f, -2.0f);
        Quaternion result = instance.add(b);
        assertEquals(expResult, result);
    }

    /**
     * Test of multiply method, of class Quaternion.
     */
    @Test
    public void testMultiply()
    {
        System.out.println("multiply");
        Quaternion b = new Quaternion(0.0f, 5.0f, 1.0f, -2.0f);
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        Quaternion expResult = new Quaternion(-5.0f, 15.0f, 5.0f, -5.0f);
        Quaternion result = instance.multiply(b);
        assertEquals(expResult, result);
        
        expResult = new Quaternion(-5.0f, 15.0f, 1.0f, -7.0f);
        result = b.multiply(instance);
        assertEquals(expResult, result);
    }

    /**
     * Test of inverse method, of class Quaternion.
     */
    @Test
    public void testInverse()
    {
        System.out.println("inverse");
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        Quaternion expResult = new Quaternion(0.3f, -0.1f, -0.0f, -0.0f);
        Quaternion result = instance.inverse();
        assertEquals(expResult, result);
    }

    /**
     * Test of divide method, of class Quaternion.
     */
    @Test
    public void testDivide()
    {
        System.out.println("divide");
        Quaternion b = new Quaternion(0.0f, 5.0f, 1.0f, -2.0f);
        Quaternion instance = new Quaternion(3.0f, 1.0f, 0.0f, 0.0f);
        Quaternion expResult = new Quaternion(0.5f, 1.5f, 0.1f, -0.7f);
        Quaternion result = instance.divide(b);
        assertEquals(expResult, result);
    }
    
}
