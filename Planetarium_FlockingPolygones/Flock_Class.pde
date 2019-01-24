class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids
  float d;
  Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }


  void run() { 
    //println();
    for (Boid b : boids) {
      b.run(boids);  // Passing the entire list of boids to each boid individually
      
      //println(b.boidOsc);
    }
  }

  void addBoid(Boid b) {
    boids.add(b);
  }

  void gravitate(float posX, float posY)
  {
    PVector pos = new PVector(posX, posY);
    for (Boid b : boids) {

      PVector desired = PVector.sub(pos, b.position);
      float dist = desired.mag();
      PVector steer = PVector.sub(desired, b.velocity);
      steer.mult(dist);  
      b.applyForce(steer);
      //println(steer);
    }
  }

  void iterativeConnect(int it)
  {
    //if(iterate > flock.boids.size())iterate = 0;
    //img.background(0);
    for(int i = 0; i<it; i++)
    {
      for (Boid b : boids)
      {
        for(Boid other : boids)
        {
          if (b!=other && b.index == i && other.index == i+1)
          {
           // println(i);
            float a = map(i,0,it,0,255);
            img.stroke(b.c, a);
            img.fill(b.c,a);
            //img.strokeWeight(2);
            if (key == '3' || mode == 3)img.line(b.position.x,b.position.y,other.position.x,other.position.y);
           if(key == '4' || mode == 4) img.ellipse(b.position.x,b.position.y,float(40-b.neighbours),float(40-b.neighbours));
          }
        }
      }
    }
  }
  
  void newRandomPolys()
   {
     for (Boid b : boids)
      {
        b.polyCount = int(random(3,maxPolys));
      }
   }
}
