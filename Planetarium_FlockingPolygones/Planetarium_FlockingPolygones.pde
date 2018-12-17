import oscP5.*;
import netP5.*;

Flock flock;
int ssAmount =5;  //die anzahl genutzer soundsurces
int boidAmount = 70;
float boidSize = 30;
float connectionDist = 100;
float maxforce  = 0.03;   // Maximum steering force
float maxspeedInit  = 3.5;  // Maximum speed
float maxspeed = maxspeedInit;
float seperationForce = 3.0;
float alignmentForce = 1.0;
float cohesionForce = 2.0;
int maxPolys = 3;
float alpha = 255;
int flagCount = 0;
//ArrayList<PVector> positions; //arraylist für die positionen der boids die zu einem cluster gehören
ArrayList [] cluster = new ArrayList[ssAmount]; //array für die cluster (definiert durch die positions der zugehörigen boids)

OscP5 oscP5;
NetAddress max;
//int index = 0;


PGraphics img;

void setup() {
  size(1080, 1080,P2D);
  img = createGraphics(width,height,P2D);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < boidAmount; i++) {
    flock.addBoid(new Boid(img.width/2, img.height/2,i));
  }
  frameRate(30);
  
  oscP5 = new OscP5(this,11000);
  max = new NetAddress("localhost",12000);
  
  OscMessage widthMessage = new OscMessage("/width");
  widthMessage.add(width);
  oscP5.send(widthMessage,max);
  
  OscMessage heightMessage = new OscMessage("/height");
  heightMessage.add(height);
  oscP5.send(heightMessage,max);
  background(0);
  for(int i=0; i< cluster.length;i++){
  cluster[i]= new ArrayList<PVector>();
  println(cluster[i].size());
  }
}


void draw() {
  img.beginDraw();
  //img.fill(255);
  img.background(0,alpha);
  img.stroke(255);
  img.strokeWeight(2);
  img.noFill();
  img.ellipse(width/2,height/2,width,height);
  flock.run();
  img.endDraw();
  //img.save("exports/Frame_" + frameCount + ".jpg");
  image(img,0,0);
  //println(maxforce,maxspeed,cohesionForce,alignmentForce,seperationForce,connectionDist,alpha);
  
  if(mousePressed && mouseButton==RIGHT)
  {maxspeed = 7;
  flock.gravitate(mouseX,mouseY);}
  else maxspeed = maxspeedInit;


}

// Add a new boid into the System
void mousePressed() {
  if(mouseButton==LEFT){
  boidAmount++;
  flock.addBoid(new Boid(mouseX, mouseY,boidAmount));}
 //flock.gravitate(mouseX,mouseY);
}



// The Flock (a list of Boid objects)

class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids
  float d;
  Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }
  

  void run() {
    for (Boid b : boids) {
      b.flag=-1;
    }   
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
    PVector pos = new PVector(posX,posY);
    for(Boid b : boids){
      
    PVector desired = PVector.sub(pos,b.position);
    float dist = desired.mag();
    PVector steer = PVector.sub(desired,b.velocity);
    steer.mult(dist);  
    b.applyForce(steer);
    //println(steer);
    }
  }
  
}



  
  void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  print("### received an osc message.");
  print(" addrpattern: "+theOscMessage.addrPattern());
  println(" typetag: "+theOscMessage.typetag());
  if(theOscMessage.checkAddrPattern("/cohesionForce")==true) {
    cohesionForce = theOscMessage.get(0).floatValue();}
  if(theOscMessage.checkAddrPattern("/alignmentForce")==true) {
   alignmentForce = theOscMessage.get(0).floatValue();}   
  if(theOscMessage.checkAddrPattern("/seperationForce")==true) {
    seperationForce = theOscMessage.get(0).floatValue();}  
  if(theOscMessage.checkAddrPattern("/maxSpeed")==true) {
    maxspeed = theOscMessage.get(0).floatValue();
    maxspeedInit = theOscMessage.get(0).floatValue();}   
  if(theOscMessage.checkAddrPattern("/maxForce")==true) {
    maxforce = theOscMessage.get(0).floatValue();}   
  if(theOscMessage.checkAddrPattern("/connectionDist")==true) {
    connectionDist = theOscMessage.get(0).floatValue()*width;} 
  if(theOscMessage.checkAddrPattern("/alpha")==true) {
    alpha = theOscMessage.get(0).floatValue();}    
}
