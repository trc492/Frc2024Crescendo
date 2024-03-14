/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492.autotasks;

import java.util.ArrayList;
import java.util.HashMap;

public class ShootParamTable
{
    public static class Params
    {
        public final String loc;
        public final double distance;
        public final double shooterVelocity;
        public final double tiltAngle;

        public Params(
            String loc, double distance, double shooterVelocity, double tiltAngle)
        {
            this.loc = loc;
            this.distance = distance;
            this.shooterVelocity = shooterVelocity;
            this.tiltAngle = tiltAngle;
        }   //Params

        @Override
        public String toString()
        {
            return "{loc=" + loc +
                   ", distance=" + distance +
                   ", shooterVel=" + shooterVelocity +
                   ", tiltAngle=" + tiltAngle + "}";
        }   //toString

    }   //class Params

    private final ArrayList<Params> paramTable;
    private final HashMap<String, Params> paramMap;

    /**
     * Constructor: Create an instance of the object.
     */
    public ShootParamTable()
    {
        paramTable = new ArrayList<>();
        paramMap = new HashMap<>();
    }   //ShootParamTable

    /**
     * This method adds an entry to the ShootParamTable sorted by distance.
     *
     * @param loc specifies the shoot location for the entry.
     * @param distance specifies the target distance.
     * @param shooterVel specifies the shooter velocity in RPS.
     * @param tiltAngle specifies the tilt angle in degrees.
     *
     * @return this instance object.
     */
    public ShootParamTable add(String loc, double distance, double shooterVel, double tiltAngle)
    {
        Params newEntry = new Params(loc, distance, shooterVel, tiltAngle);
        int insertPoint = paramTable.size();

        for (int i = 0; i < paramTable.size(); i++)
        {
            Params entry = paramTable.get(i);
            if (distance == entry.distance)
            {
                throw new RuntimeException("An entry with the same distance already exist.");
            }
            else if (distance < entry.distance)
            {
                insertPoint = i;
                break;
            }
        }

        paramTable.add(insertPoint, newEntry);
        paramMap.put(newEntry.loc, newEntry);
        return this;
    }   //add

    /**
     * This method returns the Shoot Param entry that matches the given name.
     *
     * @param loc specifies the shoot location for the entry.
     * @return shoot param entry that matches the shoot location, null if not found.
     */
    public Params get(String loc)
    {
        return paramMap.get(loc);
    }   //get

    /**
     * This method returns the Shooter velocity and tilt angle from the table entry that matches the given name.
     *
     * @param loc specifies the shoot location for the entry.
     * @return shooter velocity and tilt angle in an array, null if not found.
     */
    public double[] getShootParams(String loc)
    {
        double[] shootParams = null;
        Params params = paramMap.get(loc);

        if (params != null)
        {
            shootParams = new double[2];
            shootParams[0] = params.shooterVelocity;
            shootParams[1] = params.tiltAngle;
        }

        return shootParams;
    }   //getShootParams

    /**
     * This method returns the Shoot Param entry with the given distance. If there is no exact match, it will return
     * an entry that linearly interpolates between two entries in the table.
     *
     * @param distance specifies the distance to lookup in the table.
     * @return shoot param entry that matches the distance. If no exact match, an interpolated entry is returned.
     *         Returns null if the distance is out of range of the table entries.
     */
    public Params get(double distance)
    {
        Params foundEntry = null;

        if (paramTable.size() < 2)
        {
            throw new RuntimeException("ShootParamTable must have at least 2 entries.");
        }

        Params firstEntry = paramTable.get(0);
        Params lastEntry = paramTable.get(paramTable.size() - 1);

        if (distance <= firstEntry.distance)
        {
            // The provided distance is below the table range, extropolate.
            Params nextEntry = paramTable.get(1);
            foundEntry = new Params(
                "Extrapolated", distance, firstEntry.shooterVelocity,
                extrapolateTiltAngle(distance, firstEntry, nextEntry));
        }
        else if (distance > lastEntry.distance)
        {
            // The provided distance is above the table range, extropolate.
            Params prevEntry = paramTable.get(paramTable.size() - 2);
            foundEntry = new Params(
                "Extrapolated", distance, lastEntry.shooterVelocity,
                extrapolateTiltAngle(distance, prevEntry, lastEntry));
        }
        else
        {
            for (int i = 1; i < paramTable.size(); i++)
            {
                Params entry = paramTable.get(i);

                if (distance <= entry.distance)
                {
                    Params prevEntry = paramTable.get(i - 1);
                    foundEntry = new Params(
                        "Interpolated", distance, entry.shooterVelocity,
                        interpolateTiltAngle(distance, prevEntry, entry));
                    break;
                }
            }
        }

        return foundEntry;
    }   //get

    /**
     * This method interpolates the tilt angle with the given distance and two points of the neighboring segment
     * in the table.
     *
     * @param distance specifies the target distance.
     * @param entry1 specifies the lower entry of the neighboring segment.
     * @param entry2 specifies the upper entry of the neighboring segment.
     * @return interpolated value.
     */
    private double interpolateTiltAngle(double distance, Params entry1, Params entry2)
    {
        // TODO: interpolation should NOT be linear. It should have a tangent relationship with distance.
        double w = (distance - entry1.distance) / (entry2.distance - entry1.distance);
        double value = (1 - w) * entry1.tiltAngle + w * entry2.tiltAngle;
        return value;
    }   //interpolateTiltAngle

    /**
     * This method extrapolates the tilt angle with the given distance and the two points of the neighboring segment
     * in the table.
     *
     * @param distance specifies the target distance.
     * @param entry1 specifies the lower entry of the neighboring segment.
     * @param entry2 specifies the upper entry of the neighboring segment.
     * @return extrapolated value.
     */
    private double extrapolateTiltAngle(double distance, Params entry1, Params entry2)
    {
        // TODO: extrapolation should NOT be linear. It should have a tangent relationship with distance.
        double deltaAngle = entry2.tiltAngle - entry1.tiltAngle;
        double deltaDistance = entry2.distance - entry1.distance;
        double m = deltaAngle / deltaDistance;
        double b = entry1.tiltAngle - m*entry1.distance;
        double angle = m*distance + b;

        if (angle < 0.0)
        {
            // If extrapolated angle is negative, just return the angle of one of the two entries whoever is
            // closer to the given distance.
            double d1 = Math.abs(distance - entry1.distance);
            double d2 = Math.abs(distance - entry2.distance);
            angle = d1 < d2? entry1.tiltAngle: entry2.tiltAngle;
        }

        return angle;
    }   //extrapolateTiltAngle

}   //class ShootParamTable
