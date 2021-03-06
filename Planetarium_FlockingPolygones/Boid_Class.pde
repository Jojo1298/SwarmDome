
class Boid {

  PVector position;
  PVector velocity;
  PVector acceleration;
  float r;
  int polyCount;
  boolean visible = true;
  OscMessage boidOsc;
  int index;
  int neighbours;
  int visibleMode = 3; //0=invisble, 1 = fading out, 2 = fading in, 3 = visible
  //int alpha = 0;
  int invisibleFrame;
  

  color c;
  boolean overrideColor = false;
  
  Boid(float x, float y, int boidIndex) {
    acceleration = new PVector(0, 0);

    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));

    position = new PVector(x, y);
    r = boidSize;
    
    polyCount = int(random(3,maxPolys+1));
   
    index = boidIndex;
    
  }

  void run(ArrayList<Boid> boids) {
 if(visible)
 {
    flock(boids);
    update();
    borders();
    render();
   // integrate(boids);

   boidOsc = new OscMessage("/boid");
   boidOsc.add(new int[] {index,int(position.x),int(position.y),polyCount,neighbours});  
   oscP5.send(boidOsc,max);
 }
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    //integrate(boids);
    // Arbitrarily weight these forces
    sep.mult(seperationForce);
    ali.mult(alignmentForce);
    coh.mult(cohesionForce);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
    
    neighbours = getNeighbours(boids);
    
    if(draw)
    {
    drawLines(boids);
    }  
    //println(neighbours);
  }

  // Method to update position
  void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    position.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
    if(polyCount>maxPolys)polyCount=maxPolys;
    
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);

    // Above two lines of code below could be condensed with new PVector setMag() method
    // Not using this method until Processing.js catches up
    // desired.setMag(maxspeed);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  void render() {
      // Draw a triangle rotated in the direction of velocity
      float theta = velocity.heading()+ radians(45);
      float insideTheta = theta;
      float oldX,oldY,x,y;
      // heading2D() above is now heading() but leaving old syntax until Processing.js catches up
      x = r*cos(insideTheta);
      y =r*sin(insideTheta);
      
     if(keyPressed&&key=='s'){
       img.stroke(255); 
      img.fill(255);
    //  img.text(flag,position.x,position.y);
      img.strokeWeight(2);
      img.pushMatrix();
      img.translate(position.x, position.y);
      img.rotate(theta);
      img.beginShape(LINES);
     
       for (int i = 0; i<=polyCount; i++)
      {  
        oldX = x;
        oldY = y;
        x = r*cos(insideTheta);
        y = r*sin(insideTheta);
       insideTheta += PI/polyCount*2 ;
         img.fill(200);
        img.vertex(oldX, oldY);
        img.vertex(x,y);
      }
      img.endShape();
      img.popMatrix(); 
      } 
  }
  // Wraparound
  void borders() {
    PVector center = new PVector(img.width/2,img.height/2);
    PVector dist = PVector.sub(center,position);
    if (dist.mag() > img.width/2 || dist.mag() > img.height/2)
    {
     velocity.mult(-1);
     velocity.add(dist);
    }
    
  /*  if (position.x < -r) position.x = img.width+r;
    if (position.y < -r) position.y = img.height+r;
    if (position.x > img.width+r) position.x = -r;
    if (position.y > img.height+r) position.y = -r;*/
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = seperationDist;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)){
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, other.position);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {

      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = r*3;
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist) && (polyCount==other.polyCount)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);


      // Implement Reynolds: Steering = Desired - Velocity
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      return steer;
    } else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average position (i.e. center) of all nearby boids, calculate steering vector towards that position
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = r*3;
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist) && (polyCount==other.polyCount)) {
        sum.add(other.position); // Add position
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
     //img.ellipse(sum.x,sum.y,10,10);
      return seek(sum);  // Steer towards the position
    } else {
      return new PVector(0, 0);
    }
  }
  
  
  
  int getNeighbours (ArrayList<Boid> boids){
    float neighbordist = connectionDist;
    int Neighbours = 0;
       
    for(Boid other: boids){
      float dist = PVector.dist(position, other.position);     
      if((dist>0) && (dist<neighbordist)){        
        Neighbours++;           
      }
    }
    return Neighbours;
  }
  
  
  void drawLines(ArrayList<Boid> boids)
  {
    float neighbordist = connectionDist; 
    int Neighbours = getNeighbours(boids);
    float g = map(position.x,0,img.width,0,255);
    float b = map(position.y,0,img.height,0,255);
    
    for(Boid other: boids){
      float dist = PVector.dist(position, other.position);
      
      /*if(Neighbours==1)
      {
        visibleMode = 1;
        invisibleFrame = frameCount+50;
        visibleMode = 0;   
      }
      if(Neighbours>1 && visibleMode==0) visibleMode = 2;
      
      if(visibleMode == 0)
      {
        while(frameCount<invisibleFrame)
        {
          int preAlpha = invisibleFrame-frameCount;
          alpha = int(map(preAlpha,50,0,0,255));
        }
      }*/
      
    if((dist>0) && (dist<neighbordist)){  
      //Neighbours++;    
      float d = map(dist,1,neighbordist,0,255);
      if(!overrideColor){
      c = color(255-d,g-d,b-d);
      }
      img.stroke(c);
      float distToStrokeWeight = map(dist,0,neighbordist,5,0);
      img.strokeWeight(distToStrokeWeight);
      
        if(Neighbours>2){
        img.line(position.x,position.y,other.position.x,other.position.y);}
    }
    }
  }
}
