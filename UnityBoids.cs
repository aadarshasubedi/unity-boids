// Path Following
// Daniel Shiffman <http://www.shiffman.net>
// The Nature of Code, Spring 2009

// Boid class
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class UnityBoids : MonoBehaviour
{
    // All the usual stuff
    public Vector3 loc;
    public Vector3 vel;
    public Vector3 acc;
    private float wandertheta;


    public float r;
    public float maxforce;    // Maximum steering force
    public float maxspeed;    // Maximum speed
    public bool debug;
    public Vector3 target, dir;

    //private GameObject[] totalBoids;
    private UnityBoids[] totalBoids;

    // Constructor initialize all values

    void Awake()
    {
        //UpdateTotalBoids();
        r = 1;
        maxspeed = 0.1f;
        maxforce = 0.01f;
        acc = new Vector3(0, 0);
        vel = new Vector3(maxspeed, 1, 1);
    }



    // A function to deal with path following and separation
    public void applyForces(Path path)
    {
        // Follow path force
        Vector3 f = follow(path);
        // Separate from other boids force
        Vector3 s = separate();
        // Arbitrary weighting
        f *= 3;
        s *= 2;
        // Accumulate in acceleration
        acc += f;
        acc += s;
    }


    // We accumulate a new acceleration each time based on three rules
    public void flock()
    {
        Vector3 sep = separate();   // Separation
        Vector3 ali = align();      // Alignment
        Vector3 coh = cohesion();   // Cohesion
        // Arbitrarily weight these forces
        sep *= (1.5f);
        ali *= (1.0f);
        coh *= (1.0f);
        // Add the force vectors to acceleration
        acc += (sep);
        acc += (ali);
        acc += (coh);

    }

    // This function implements Craig Reynolds' path following algorithm
    // http://www.red3d.com/cwr/steer/PathFollow.html

    public Vector3 follow(Path p)
    {

        // Predict location 25 (arbitrary choice) frames ahead
        Vector3 predict = vel;
        predict.Normalize();
        predict *= 25;
        Vector3 predictLoc = loc + predict;

        // Now we must find the normal to the path from the predicted location
        // We look at the normal for each line segment and pick out the closest one


        float record = 1000000;  // Start with a very high record distance that can easily be beaten

        // Loop through all points of the path
        for (int i = 0; i < p.points.Length; i++)
        {

            // Look at a line segment
            Vector3 a = (Vector3)p.points[i];
            Vector3 b = (Vector3)p.points[(i + 1) % p.points.Length];  // Path wraps around

            // Get the normal point to that line
            Vector3 normal = getNormalPoint(predictLoc, a, b);

            // Check if normal is on line segment
            float da = Vector3.Distance(normal, a);
            float db = Vector3.Distance(normal, b);
            Vector3 line = b - a;
            // If it's not within the line segment, consider the normal to just be the end of the line segment (point b)
            if (da + db > line.magnitude + 1)
            {
                normal = b;
                // If we're at the end we really want the next line segment for looking ahead
                a = (Vector3)p.points[(i + 1) % p.points.Length];
                b = (Vector3)p.points[(i + 2) % p.points.Length];  // Path wraps around
                line = b - a;
            }

            // How far away are we from the path?
            float d = Vector3.Distance(predictLoc, normal);
            // Did we beat the record and find the closest line segment?
            if (d < record)
            {
                record = d;
                // If so the target we want to steer towards is the normal
                target = normal;

                // Look at the direction of the line segment so we can seek a little bit ahead of the normal
                dir = line;
                dir.Normalize();
                // This is an oversimplification
                // Should be based on distance to path & velocity
                dir *= 25;
            }
        }

        // Only if the distance is greater than the path's radius do we bother to steer
        if (record > p.radius || vel.magnitude < 0.1f)
        {
            target += dir;
            return steer(target, false);
        }
        else
        {
            return new Vector3(0, 0);
        }
    }

    // A function to get the normal point from a point (p) to a line segment (a-b)
    // This function could be optimized to make fewer new Vector objects
    Vector3 getNormalPoint(Vector3 p, Vector3 a, Vector3 b)
    {
        // Vector from a to p
        Vector3 ap = p - a;
        // Vector from a to b
        Vector3 ab = b - a;
        ab.Normalize(); // Normalize the line
        // Project vector "diff" onto line by using the dot product

        ab *= (Vector3.Dot(ap, ab));
        Vector3 normalPoint = a + ab;
        return normalPoint;
    }

    // Separation
    // Method checks for nearby boids and steers away
    Vector3 separate()
    {
        float desiredseparation = r * 2;
        Vector3 steer = new Vector3(0, 0, 0);
        int count = 0;
        // For every boid in the system, check if it's too close
        for (int i = 0; i < totalBoids.Length; i++)
        {
            float d = Vector3.Distance(loc, totalBoids[i].loc);

            // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
            if ((d > 0) && (d < desiredseparation))
            {
                // Calculate vector pointing away from neighbor
                Vector3 diff = loc - totalBoids[i].loc;
                diff.Normalize();
                diff /= d;        // Weight by distance
                steer += diff;
                count++;            // Keep track of how many
            }
        }
        // Average -- divide by how many
        if (count > 0)
        {
            steer /= ((float)count);
        }

        // As long as the vector is greater than 0
        if (steer.magnitude > 0)
        {
            // Implement Reynolds: Steering = Desired - Velocity
            steer.Normalize();
            steer *= maxspeed;
            steer -= vel;
            //steer.limit(maxforce);
            steer = Vector3.ClampMagnitude(steer, maxforce);
        }
        return steer;
    }


    Vector3 align()
    {
        float neighbordist = 50.0f;
        Vector3 steer = new Vector3(0, 0, 0);
        int count = 0;
        for (int i = 0; i < totalBoids.Length; i++)
        {

            float d = Vector3.Distance(loc, totalBoids[i].loc);
            if ((d > 0) && (d < neighbordist))
            {
                steer += totalBoids[i].vel;
                count++;
            }
        }
        if (count > 0)
        {
            steer /= ((float)count);
        }

        // As long as the vector is greater than 0
        if (steer.magnitude > 0)
        {
            // Implement Reynolds: Steering = Desired - Velocity
            steer.Normalize();
            steer *= (maxspeed);
            steer -= (vel);
            steer = Vector3.ClampMagnitude(steer, maxforce);
        }
        return steer;
    }


    // Cohesion
    // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
    Vector3 cohesion()
    {
        float neighbordist = 50.0f;
        Vector3 sum = new Vector3(0, 0, 0);   // Start with empty vector to accumulate all locations
        int count = 0;
        for (int i = 0; i < totalBoids.Length; i++)
        {
            UnityBoids other = totalBoids[i];
            float d = Vector3.Distance(loc, other.loc);
            if ((d > 0) && (d < neighbordist))
            {
                sum += (other.loc); // Add location
                count++;
            }
        }
        if (count > 0)
        {
            sum /= ((float)count);
            return steer(sum, false);  // Steer towards the location
        }
        return sum;
    }

    // Method to update location

    public void Update()
    {

        // Update velocity
        // Debug.Log(acc.x + "-" + accOld.x);
        //this.transform.rotation = Quaternion.FromToRotation(acc/1000,acc*1000);

        vel += acc * (Time.deltaTime * 2);

        // Limit speed
        //vel.limit(maxspeed);
        vel = Vector3.ClampMagnitude(vel, maxspeed);
        loc += vel;
        // Reset accelertion to 0 each cycle

        acc *= 0;

        if (vel.magnitude != 0)
        {
            float rotateY = Mathf.Atan2(-vel.z, vel.x) * Mathf.Rad2Deg;
            float rotateZ = Mathf.Asin(vel.y / vel.magnitude) * Mathf.Rad2Deg;
            this.transform.rotation = Quaternion.Euler(0, rotateY, rotateZ);
        }

        //flock();
        // borders();
        //UpdateTotalBoids();
        this.gameObject.transform.position = new Vector3(loc.x, loc.y, loc.z);


    }

    Vector3 Direction(Vector3 from, Vector3 to)
    {
        Vector3 retorno = to - from;
        return retorno.normalized;
    }


    // A method that calculates a steering vector towards a target
    // Takes a second argument, if true, it slows down as it approaches the target
    Vector3 steer(Vector3 target, bool slowdown)
    {
        Vector3 steer;  // The steering vector
        Vector3 desired = target - loc;  // A vector pointing from the location to the target
        float d = desired.magnitude; // Distance from the target is the magnitude of the vector
        // If the distance is greater than 0, calc steering (otherwise return zero vector)
        if (d > 0)
        {
            // Normalize desired
            desired.Normalize();
            // Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
            if ((slowdown) && (d < 100.0f)) desired *= (maxspeed * (d / 100.0f)); // This damping is somewhat arbitrary
            else desired *= maxspeed;
            // Steering = Desired minus Velocity
            steer = desired - vel;

            //steer.limit(maxforce);  // Limit to maximum steering force
            steer = Vector3.ClampMagnitude(steer, maxforce);
        }
        else
        {
            steer = new Vector3(0, 0);
        }
        return steer;
    }


    //**********************************************************************
    public void seek(Vector3 target)
    {
        acc += (steer(target, false));
    }

    public void arrive(Vector3 target)
    {
        acc += (steer(target, true));
    }

    public void wander()
    {
        //float wanderR = 16.0f;         // Radius for our "wander circle"
        //float wanderD = 60.0f;         // Distance for our "wander circle"
        //float change = 0.25f;
        float wanderR = 16f;         // Radius for our "wander circle"
        float wanderD = 60f;         // Distance for our "wander circle"
        float change = 0.25f;
        wandertheta += Random.Range(-change, change);     // Randomly change wander theta

        // Now we have to calculate the new location to steer towards on the wander circle
        Vector3 circleloc = vel;  // Start with velocity
        circleloc.Normalize();            // Normalize to get heading
        circleloc *= (wanderD);          // Multiply by distance
        circleloc += (loc);               // Make it relative to boid's location

        Vector3 circleOffSet = new Vector3(wanderR * Mathf.Cos(wandertheta), wanderR * Mathf.Sin(wandertheta));
        Vector3 target = (circleloc + circleOffSet);
        acc += (steer(target, false));  // Steer towards it

        // Render wandering circle, etc. 
        //if (debug) drawWanderStuff(loc, circleloc, target, wanderR);

        Vector3 sep = separate();   // Separation
        Vector3 coh = cohesion();   // Cohesion
        // Arbitrarily weight these forces
        sep *= (1.5f);
        coh *= (1.0f);
        // Add the force vectors to acceleration
        acc += (sep);
        acc += (coh);

    }


    // Wraparound
    public void borders()
    {

        if (loc.x < -50) vel.x += maxforce / 2;
        if (loc.x > 50) vel.x -= maxforce / 2;

        if (loc.y < -50) vel.y += maxforce / 2;
        if (loc.y > 50) vel.y -= maxforce / 2;

        if (loc.z < -50) vel.z += maxforce / 2;
        if (loc.z > 50) vel.z -= maxforce / 2;

        /*
                if (loc.x < -20) acc.x = acc.x + maxforce;
                if (loc.x > 20) acc.x = acc.x - maxforce;

                if (loc.y < -20) acc.y = acc.y + maxforce;
                if (loc.y > 20) acc.y = acc.y - maxforce;

                if (loc.z < -20) acc.z = acc.z + maxforce;
                if (loc.z > 20) acc.z = acc.z - maxforce;
         */
    }

    //**************************************

    public void UpdateTotalBoids(UnityBoids[] total)
    {
        totalBoids = total;
        //totalBoids = GameObject.FindObjectsOfType(typeof(UnityBoids)) as UnityBoids[];
        //totalBoids = (totalBoids[])GameObject.FindObjectsOfType(typeof(UnityBoids)).to;
    }

    void OnDrawGizmos()
    {
        //Debug.Log("----------------------------");
        Gizmos.color = new Color(0.3f, 0.3f, 1.3f, 0.2f);
        Gizmos.DrawWireSphere(this.transform.position, r);

    }

    public class Path
    {
        public Vector3[] points;
        public float radius;

        public Path() {
            points = new Vector3[0];
            radius = 0f;
        }

        public void AddPoint(Vector3 point)
        {
            if (points.Length > 0)
            {
                Vector3[] pointsOld = points;
                points = new Vector3[pointsOld.Length + 1];
                pointsOld.CopyTo(points, 0);
                points[points.Length - 1] = point;
            }
            else
            {
                points = new Vector3[1];
                points[0] = point;
            }

            //temp_points.co
            Debug.Log(points.Length);
        }
    };

}