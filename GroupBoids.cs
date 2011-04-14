using UnityEngine;
using System.Collections;

public class GroupBoids : MonoBehaviour
{

    // Use this for initialization
    float raio = 1f;
    float maxforce, maxspeed;
    public UnityBoids[] totalBoids;
    UnityBoids.Path caminho = new UnityBoids.Path();
    void Start()
    {
        for (int i = 0; i < 100; i++)
        {
            GameObject goBoid = Instantiate(GameObject.Find("boid")) as GameObject;
            (goBoid.GetComponent(typeof(UnityBoids)) as UnityBoids).loc = new Vector3(Random.Range(-4, 4), Random.Range(-4, 4), Random.Range(-4, 4));
        }
    }

    void OnGUI() {
        raio = GUILayout.HorizontalSlider(raio,0,10,GUILayout.Width(200));
        maxforce = GUILayout.HorizontalSlider(maxforce, 0, 0.1f, GUILayout.Width(200));
        maxspeed = GUILayout.HorizontalSlider(maxspeed, 0, 1, GUILayout.Width(200));
        if (GUILayout.Button("organizar")) {
			totalBoids = GameObject.FindObjectsOfType(typeof(UnityBoids)) as UnityBoids[];
         
        }
        if (GUILayout.Button("copiar")) {
            caminho.AddPoint(new Vector3(0, 1, 2));
        }
    }

    // Update is called once per frame
    void Update()
    {


        for (int i = 0; i < totalBoids.Length; i++)
        {
            UnityBoids boid = totalBoids[i] as UnityBoids;
            
            // boid.flock();
            boid.r = raio;
            boid.maxforce = maxforce;
            boid.maxspeed = maxspeed;

            boid.UpdateTotalBoids(totalBoids);
            boid.flock();
            // boid.applyForces(caminho);
            boid.borders();

        }
        // Debug.Log(Camera.main.ScreenToWorldPoint(Input.mousePosition));


        

        if (Input.GetMouseButtonUp(2)) {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            Plane plane = new Plane(Vector3.up, Vector3.zero);
            float teste = 100f;
            if (plane.Raycast(ray, out teste))
            {
                caminho.AddPoint(ray.GetPoint(teste));
            }
            
        }


    }

    void OnDrawGizmos(){
        Gizmos.color = new Color(0.1f, 1f, 0.5f, 0.1f);
        Gizmos.DrawCube(new Vector3(0, 0, 0), new Vector3(100, 100, 100));

        Gizmos.color = new Color(0.1f, 1f, 0.5f, 0.3f);
        if (caminho.points.Length > 0) {
            for (int i = 1; i < caminho.points.Length; i++) {
                Gizmos.DrawWireCube(caminho.points[i - 1], new Vector3(0.4f, 0.4f, 0));
                Gizmos.DrawLine(caminho.points[i-1], caminho.points[i]);
            }
        }
    }
}

