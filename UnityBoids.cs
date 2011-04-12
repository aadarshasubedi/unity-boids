// teste de edicao
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
    private Vector2 loc;
    private Vector2 vel;
    private Vector2 acc;
    private float wandertheta;

    public float r;
    public float maxforce;    // Maximum steering force
    public float maxspeed;    // Maximum speed
    public bool debug;
    public Vector2 target, dir;

    private GameObject[] totalBoids;

    // Constructor initialize all values

    void Awake()
    {
        UpdateTotalBoids();
    }

    public void Initialize(Vector2 l, float ms, float mf)
    {
        loc = l;
        r = 4;
        maxspeed = ms;
        maxforce = mf;
        acc = new Vector2(0, 0);
        vel = new Vector2(maxspeed, 0);
    }

    // A function to deal with path following and separation
    public void applyForces(GameObject[] boids, Path path)
    {
        // Follow path force
        Vector2 f = follow(path);
        // Separate from other boids force
        Vector2 s = separate();
        // Arbitrary weighting
        f *= 3;
        s *= 1;
        // Accumulate in acceleration
        acc += f;
        acc += s;
    }


    // We accumulate a new acceleration each time based on three rules
    public void flock()
    {
        Vector2 sep = separate();   // Separation
        Vector2 ali = align();      // Alignment
        Vector2 coh = cohesion();   // Cohesion
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
    public Vector2 follow(Path p)
    {

        // Predict location 25 (arbitrary choice) frames ahead
        Vector2 predict = vel;
        predict.Normalize();
        predict *= 25;
        Vector2 predictLoc = loc + predict;

        // Now we must find the normal to the path from the predicted location
        // We look at the normal for each line segment and pick out the closest one


        float record = 1000000;  // Start with a very high record distance that can easily be beaten

        // Loop through all points of the path
        for (int i = 0; i < p.points.Count; i++)
        {

            // Look at a line segment
            Vector2 a = (Vector2)p.points[i];
            Vector2 b = (Vector2)p.points[(i + 1) % p.points.Count];  // Path wraps around

            // Get the normal point to that line
            Vector2 normal = getNormalPoint(predictLoc, a, b);

            // Check if normal is on line segment
            float da = Vector2.Distance(normal, a);
            float db = Vector2.Distance(normal, b);
            Vector2 line = b - a;
            // If it's not within the line segment, consider the normal to just be the end of the line segment (point b)
            if (da + db > line.magnitude + 1)
            {
                normal = b;
                // If we're at the end we really want the next line segment for looking ahead
                a = (Vector2)p.points[(i + 1) % p.points.Count];
                b = (Vector2)p.points[(i + 2) % p.points.Count];  // Path wraps around
                line = b - a;
            }

            // How far away are we from the path?
            float d = Vector2.Distance(predictLoc, normal);
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
            return new Vector2(0, 0);
        }
    }

    // A function to get the normal point from a point (p) to a line segment (a-b)
    // This function could be optimized to make fewer new Vector objects
    Vector2 getNormalPoint(Vector2 p, Vector2 a, Vector2 b)
    {
        // Vector from a to p
        Vector2 ap = p - a;
        // Vector from a to b
        Vector2 ab = b - a;
        ab.Normalize(); // Normalize the line
        // Project vector "diff" onto line by using the dot product

        ab *= (Vector2.Dot(ap, ab));
        Vector2 normalPoint = a + ab;
        return normalPoint;
    }

    // Separation
    // Method checks for nearby boids and steers away
    Vector2 separate()
    {
        float desiredseparation = r * 2;
        Vector2 steer = new Vector2(0, 0);
        int count = 0;
        // For every boid in the system, check if it's too close
        for (int i = 0; i < totalBoids.Length; i++)
        {
            float d = Vector2.Distance(loc, (totalBoids[i].GetComponent(typeof(Boid)) as Boid).loc);

            // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
            if ((d > 0) && (d < desiredseparation))
            {
                // Calculate vector pointing away from neighbor
                Vector2 diff = loc - (((GameObject)totalBoids[i]).GetComponent(typeof(Boid)) as Boid).loc;
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
            steer = Vector2.ClampMagnitude(steer, maxforce);
        }
        return steer;
    }


    Vector2 align()
    {
        float neighbordist = 50.0f;
        Vector2 steer = new Vector2(0, 0);
        int count = 0;
        for (int i = 0; i < totalBoids.Length; i++)
        {

            float d = Vector2.Distance(loc, (((GameObject)totalBoids[i]).GetComponent(typeof(Boid)) as Boid).loc);
            if ((d > 0) && (d < neighbordist))
            {
                steer += ((((GameObject)totalBoids[i]).GetComponent(typeof(Boid)) as Boid).vel);
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
            steer = Vector2.ClampMagnitude(steer, maxforce);
        }
        return steer;
    }


    // Cohesion
    // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
    Vector2 cohesion()
    {
        float neighbordist = 50.0f;
        Vector2 sum = new Vector2(0, 0);   // Start with empty vector to accumulate all locations
        int count = 0;
        for (int i = 0; i < totalBoids.Length; i++)
        {
            Boid other = (((GameObject)totalBoids[i]).GetComponent(typeof(Boid)) as Boid);
            float d = Vector2.Distance(loc, other.loc);
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
    void Update()
    {
        
        // Update velocity
        vel += acc;
        // Limit speed
        //vel.limit(maxspeed);
        vel = Vector2.ClampMagnitude(vel, maxspeed);
        loc += vel;
        // Reset accelertion to 0 each cycle
        acc *= 0;

        //flock();

        //borders();

        //this.gameObject.transform.position = new Vector3(loc.x, 0, loc.y);
        //Debug.Log(new Vector3(loc.x, 0, loc.y));

        //this.gameObject.transform.position = new Vector3(Mathf.Sin(Time.time), Mathf.Cos(Time.time));
        this.gameObject.transform.position = new Vector3(loc.x,0,loc.y);

        //float theta = heading2D(vel) + (Mathf.Deg2Rad * 90f);
        //float theta = heading2D(vel);
        //Debug.Log(Mathf.Rad2Deg * theta);
        //Debug.Log((Mathf.Deg2Rad * 90f));
        //Debug.Log(Direction(new Vector2(0,0),vel));

    }

    Vector2 Direction(Vector2 from ,Vector2 to)
    {
        Vector2 retorno = to - from;
        return retorno.normalized;
    }


    // A method that calculates a steering vector towards a target
    // Takes a second argument, if true, it slows down as it approaches the target
    Vector2 steer(Vector2 target, bool slowdown)
    {
        Vector2 steer;  // The steering vector
        Vector2 desired = target - loc;  // A vector pointing from the location to the target
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
            steer = Vector2.ClampMagnitude(steer, maxforce);
        }
        else
        {
            steer = new Vector2(0, 0);
        }
        return steer;
    }


    //**********************************************************************
    public void seek(Vector2 target)
    {
        acc += (steer(target, false));
    }

    public void arrive(Vector2 target)
    {
        acc += (steer(target, true));
    }

    public void wander()
    {
        float wanderR = 16.0f;         // Radius for our "wander circle"
        float wanderD = 60.0f;         // Distance for our "wander circle"
        float change = 0.25f;
        wandertheta += Random.Range(-change, change);     // Randomly change wander theta

        // Now we have to calculate the new location to steer towards on the wander circle
        Vector2 circleloc = vel;  // Start with velocity
        circleloc.Normalize();            // Normalize to get heading
        circleloc *= (wanderD);          // Multiply by distance
        circleloc += (loc);               // Make it relative to boid's location

        Vector2 circleOffSet = new Vector2(wanderR * Mathf.Cos(wandertheta), wanderR * Mathf.Sin(wandertheta));
        Vector2 target = (circleloc + circleOffSet);
        acc += (steer(target, false));  // Steer towards it

        // Render wandering circle, etc. 
        //if (debug) drawWanderStuff(loc, circleloc, target, wanderR);

        Vector2 sep = separate();   // Separation
        Vector2 coh = cohesion();   // Cohesion
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
        if (loc.x < -50) acc.x = acc.x + 0.01f;
        if (loc.x > 50) acc.x = acc.x - 0.01f;

        if (loc.y < -50) acc.y = acc.y + 0.01f;
        if (loc.y > 50) acc.y = acc.y - 0.01f;

        //if (loc.x < -r) loc.x = width + r;    
        //if (loc.y < -r) loc.y = height+r;     
        //if (loc.x > width + r) loc.x = -r;    
        //if (loc.y > height+r) loc.y = -r;     
    }

    //**************************************

    public void UpdateTotalBoids()
    {
        totalBoids = new GameObject[GameObject.FindGameObjectsWithTag("Boid").Length];
        totalBoids = GameObject.FindGameObjectsWithTag("Boid");
    }

    void OnDrawGizmos()
    {
        //Debug.Log("----------------------------");
         Gizmos.DrawWireSphere(new Vector3(loc.x, 0, loc.y), r);
         //Debug.Log(vel);

    }


}