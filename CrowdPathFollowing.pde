
/* 2016.12.03 */

MulPointsPath mpath ;

ArrayList<Vehicle> vehicles ;

void setup() {
  
  size(640 , 360) ;
  
  mpath = new MulPointsPath() ;
  float offset = 30 ;
  mpath.addPoint( new PVector(offset       , offset) ) ;
  mpath.addPoint( new PVector(width-offset , offset) ) ;
  mpath.addPoint( new PVector(width-offset , height-offset) ) ;
  mpath.addPoint( new PVector(width/2      , height-offset*3) ) ;
  mpath.addPoint( new PVector(offset       , height-offset) ) ;
  
  vehicles = new ArrayList<Vehicle>() ;  
  for(int i = 0 ; i < 20 ; i++) {
    vehicles.add( 
                  new Vehicle(
                      new PVector(random(width) , random(height))
                  ) 
                  ) ;
  }
  
}

void draw() {
  
  background(200) ;
  
  mpath.display() ;
  
  for(Vehicle v : vehicles) {
    v.applyBehaviors( vehicles , mpath ) ;
    v.run() ;
  }
  
}
