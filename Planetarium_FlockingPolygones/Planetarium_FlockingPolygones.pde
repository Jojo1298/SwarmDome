import oscP5.*;
import netP5.*;

Flock flock;
int ssAmount =5;  //die anzahl genutzer soundsurces
int boidAmount = 50;
float boidSize = 30;
float connectionDist;
float seperationDist;
float maxforceInit = 0.03;
float maxforce  = maxforceInit;   // Maximum steering force
float maxspeedInit  = 3.5;  // Maximum speed
float maxspeed = maxspeedInit;
float seperationForce = 3.0;
float alignmentForce = 1.0;
float cohesionForce = 2.0;
int maxPolys = 5;
float alpha = 255;
int flagCount = 0;
boolean record = false;
int mode = 1;

OscP5 oscP5;
NetAddress max;
//int index = 0;

//effect counter for different mouseclicks
int fxCount = 0;
int iterate = 0;
//boolean for fx
boolean draw = true;
boolean connect = false;

String frameSuf;

PGraphics img;

void setup() {
  size(1080, 1080 , P2D);
  img = createGraphics(width, height, P2D);
  img.smooth(8);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < boidAmount; i++) {
    flock.addBoid(new Boid(img.width/2, img.height/2, i));
  }
  frameRate(30);
  
  connectionDist = width/10;
  seperationDist = boidSize*2;

  oscP5 = new OscP5(this, 11000);
  max = new NetAddress("localhost", 12000);

  OscMessage widthMessage = new OscMessage("/width");
  widthMessage.add(width);
  oscP5.send(widthMessage, max);

  OscMessage heightMessage = new OscMessage("/height");
  heightMessage.add(height);
  oscP5.send(heightMessage, max);
  background(0);
}


void draw() {
  img.beginDraw();
  //img.fill(255);
  img.background(0, alpha);
  img.stroke(255);
  img.strokeWeight(2);
  img.noFill();
  img.ellipse(width/2, height/2, width, height);
  flock.run();
  if(connect)
    {
      //frameRate(10);
      flock.iterativeConnect(iterate);
      iterate ++;
    }
  img.endDraw();
  if(frameCount<10)frameSuf = "0000";
  else if(frameCount<100)frameSuf ="000";
  else if (frameCount<1000)frameSuf="00";
  else if (frameCount<10000)frameSuf="0";
  else {frameSuf ="";}
  if(record)img.save("exports/Frame_"+frameSuf+ frameCount+".tga");
  image(img, 0, 0);
  //println(maxforce,maxspeed,cohesionForce,alignmentForce,seperationForce,connectionDist,alpha);

  if (mousePressed && mouseButton==RIGHT)
  {
    maxspeed = 7;
    flock.gravitate(mouseX, mouseY);
  } else maxspeed = maxspeedInit;
}

//nice FX Stuff
void mouseClicked() {

  if (mouseButton==LEFT) {
   

      boidAmount++;
      flock.addBoid(new Boid(mouseX, mouseY, boidAmount));
  
  }

  //flock.gravitate(mouseX,mouseY);
}

void keyPressed()
{
  if(key=='r')record=!record;
  
  if(key=='1')
  {
    maxspeed = maxspeedInit;
    maxforce = maxforceInit;
    draw = true;
    connect = false;
    for (Boid b : flock.boids)
         {
           b.overrideColor = false;
         }
  }
  
   if(key=='2')
  {
    maxforce = maxforceInit;
    draw = true;
    maxspeed = 0;
      for (float i = maxforce; i>0; i-=.002)
      {
        maxforce -= i;
      }
      for (Boid b : flock.boids)
       {
         float g = map(b.position.x,0,width,0,255);
         float r = map(b.position.y,0,height,0,255);
         b.overrideColor = true;
         b.c = color(r-10,180-b.getNeighbours(flock.boids),g);
       }
  }
  
    if(key=='3')
  {
    maxspeed = 0;
    maxforce = 0.0008;
    iterate = 0;
    draw = false;
        for (Boid b : flock.boids)
         {
           b.overrideColor = false;
         }
      connect = true;
  }
  
  if(key=='4')
  {
    iterate = 0;
    maxforce = 0.001;
    connect = true;
  }
  
}

void checkMode()
{

  if(mode==1)
  {
    maxspeed = maxspeedInit;
    maxforce = maxforceInit;
    draw = true;
    connect = false;
    for (Boid b : flock.boids)
         {
           b.overrideColor = false;
         }
  }
  
   if(mode==2)
  {
    maxforce = maxforceInit;
    draw = true;
    maxspeed = 0;
      for (float i = maxforce; i>0; i-=.002)
      {
        maxforce -= i;
      }
      for (Boid b : flock.boids)
       {
         float g = map(b.position.x,0,width,0,255);
         float r = map(b.position.y,0,height,0,255);
         b.overrideColor = true;
         b.c = color(r-10,180-b.getNeighbours(flock.boids),g);
       }
  }
  
    if(mode==3)
  {
    maxspeed = 0;
    maxforce = 0.0008;
    iterate = 0;
    draw = false;
        for (Boid b : flock.boids)
         {
           b.overrideColor = false;
         }
      connect = true;
  }
  
  if(mode==4)
  {
    iterate = 0;
    maxforce = 0.001;
    connect = true;
  }
  
}


void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  print("### received an osc message.");
  print(" addrpattern: "+theOscMessage.addrPattern());
  println(" typetag: "+theOscMessage.typetag());
  if (theOscMessage.checkAddrPattern("/cohesionForce")==true) {
    cohesionForce = theOscMessage.get(0).floatValue();
  }
  if (theOscMessage.checkAddrPattern("/alignmentForce")==true) {
    alignmentForce = theOscMessage.get(0).floatValue();
  }   
  if (theOscMessage.checkAddrPattern("/seperationForce")==true) {
    seperationForce = theOscMessage.get(0).floatValue();
  }  
  if (theOscMessage.checkAddrPattern("/maxSpeed")==true) {
    maxspeed = theOscMessage.get(0).floatValue();
    maxspeedInit = theOscMessage.get(0).floatValue();
  }   
  if (theOscMessage.checkAddrPattern("/maxForce")==true) {
    maxforce = theOscMessage.get(0).floatValue();
  }   
  if (theOscMessage.checkAddrPattern("/connectionDist")==true) {
    connectionDist = theOscMessage.get(0).floatValue()*width;
  } 
  if (theOscMessage.checkAddrPattern("/alpha")==true) {
    alpha = theOscMessage.get(0).floatValue();
  }
  if (theOscMessage.checkAddrPattern("/polys")==true) {
   maxPolys = theOscMessage.get(0).intValue();
   flock.newRandomPolys();
  }
  if (theOscMessage.checkAddrPattern("/mode")==true) {
   mode = theOscMessage.get(0).intValue(); 
   println(mode);
   checkMode();
  }
}
