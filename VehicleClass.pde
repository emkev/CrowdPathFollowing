
/* 2016.12.03 */

class Vehicle {
  
  PVector location ; 
  PVector velocity ; 
  PVector acceleration ; 

  float r ; 
  float maxSpeed ; 
  float maxForce ; 

  Vehicle(PVector l)
  {
    r = 3.0 ; 
    maxSpeed = 2.0 ; 
    maxForce = 0.1 ; 

    location = l.get() ; 
    velocity = new PVector(maxSpeed , 0) ; 
    acceleration = new PVector(0 , 0) ; 

  }

  void run() {
    update() ;
    bordersForCrowd() ;
    display() ;
  }
  
  void update()
  {
    velocity.add(acceleration) ; 
    velocity.limit(maxSpeed) ; 
    location.add(velocity) ; 
    acceleration.mult(0) ; 
  }

  void applyForce(PVector force)
  {
    acceleration.add(force) ; 
  }

  void display()
  {
    float theta = velocity.heading2D() + PI / 2 ; 
    fill(175) ; 
    stroke(0) ; 
    pushMatrix() ; 
    translate(location.x , location.y) ; 
    rotate(theta) ; 
    beginShape() ; 
    vertex(0 , -r * 2) ; 
    vertex(-r , r * 2) ; 
    vertex(r , r * 2) ; 
    endShape(CLOSE) ; 
    popMatrix() ; 
  }

  void follow(Path path) {
    
    PVector predict = velocity.get() ;
    predict.normalize();
    predict.mult(25);
    PVector predictLoc = PVector.add( location , predict ) ;
    
    PVector a = PVector.sub( predictLoc , path.start ) ;
    PVector b = PVector.sub( path.end , path.start ) ;
    
    float theta = PVector.angleBetween( a , b ) ;
    b.normalize();
    b.mult( a.mag() * cos(theta) ) ;
    PVector normalPoint = PVector.add( path.start , b ) ;
    
    PVector dir = PVector.sub( path.end , path.start ) ;
    dir.normalize();
    dir.mult(10);
    PVector target = PVector.add( normalPoint , dir ) ;
    
    float distance = PVector.dist( predictLoc , normalPoint ) ;
    if( distance > path.radius ) {
      seek( target ) ;
    }
  }

  void seek(PVector target) {
    
    PVector desired = PVector.sub( target , location ) ;

    if( desired.mag() == 0 ) return ;
    
    desired.normalize() ;
    desired.mult( maxSpeed ) ;
    
    PVector steer = PVector.sub( desired , velocity ) ;
    steer.limit( maxForce ) ;
    
    applyForce( steer ) ;
    
  }
  
  PVector seekSteering(PVector target) {
    
    PVector desired = PVector.sub( target , location ) ;

    if( desired.mag() == 0 ) 
      return new PVector(0 , 0) ;
    
    desired.normalize() ;
    desired.mult( maxSpeed ) ;
    
    PVector steer = PVector.sub( desired , velocity ) ;
    steer.limit( maxForce ) ;
    
    return steer ;
    
  }

  PVector followMulPointsSteering(MulPointsPath path) {
    
    PVector predict = velocity.get() ;
    predict.normalize();
    predict.mult(25);
    PVector predictLoc = PVector.add( location , predict ) ;

    PVector normal = null ;
    PVector target = null ;
    float record = 1000000.0 ;
    
    for(int i = 0 ; i < path.points.size()-1 ; i++) {
      
      PVector ps = path.points.get(i) ;
      PVector pe = path.points.get(i+1) ;
      PVector normalPo = getNormalPoint( predictLoc , ps , pe ) ;
      
      // It is necessary . 
      // If... , using a path endpoint as the normal point .
      if( normalPo.x < ps.x || normalPo.x > pe.x ) {
        normalPo = pe.get() ;
      }
      
      float dist = PVector.dist( predictLoc , normalPo ) ;

      if( dist < record ) {
        
        record = dist ;
        normal = normalPo ;
        
        PVector dir = PVector.sub( pe , ps ) ;
        dir.normalize() ;
        dir.mult(10) ;
        // forward from normalPo , for the target
        PVector targetPo = PVector.add( normalPo , dir ) ;

        target = targetPo ;     
        
      } /*  if( dist < record )  */
        
    } /*  for(int i = 0 ; i < path.points.size()-1 ; i++)  */
    
    if( record > path.radius ) {
      return seekSteering( target ) ;
    }
    else {
      return new PVector(0 , 0) ;
    }
    
  }
  
  PVector getNormalPoint(PVector predictLocation , PVector pathStart , 
                         PVector pathEnd ) {
                           
    PVector ls = PVector.sub( predictLocation , pathStart ) ;
    PVector es = PVector.sub( pathEnd         , pathStart ) ;
    
    float theta = PVector.angleBetween( ls , es ) ;
    
    es.normalize() ;
    es.mult( ls.mag() * cos(theta) ) ;
    
    PVector normalPoint = PVector.add( pathStart , es ) ;
    
    return normalPoint ;
  }
  
  PVector separate ( ArrayList<Vehicle> vhs ) {
    
    float desiredSeparation = r * 2 ;
    int count = 0 ;
    PVector steer = new PVector(0 , 0) ;
    float distance = 0.0 ;
    PVector diff = new PVector(0 , 0) ;
    PVector sum = new PVector(0 , 0) ;
    
    for( int i = 0 ; i < vhs.size() ; i++ ) {
      
      distance = PVector.dist( location , vhs.get(i).location ) ;
      
      if( distance > 0 && distance < desiredSeparation ) {
        diff = PVector.sub( location , vhs.get(i).location ) ;
        diff.normalize() ;
        sum.add( diff ) ;
        count++ ;
      }
      
    } /*  for( int i = 0 ; i < vhs.size() ; i++ ) */
    
    if( count > 0 ) {
      sum.div( count ) ;
      sum.normalize() ;
      sum.mult( maxSpeed ) ;
      
      steer = PVector.sub( sum , velocity ) ;
      steer.limit( maxForce ) ;      
    } /*  if( count > 0 )  */
    
    return steer ;
  }
  
  void applyBehaviors ( ArrayList vhs , MulPointsPath mpath ) {
    
    PVector followForce   = followMulPointsSteering( mpath ) ;
    PVector separateForce = separate( vhs ) ;
    
    followForce.mult(3) ;
    separateForce.mult(1.5) ;
    
    applyForce(followForce) ;
    applyForce(separateForce) ;
    
  }
  
  void borders(Path path) {
    
    if( location.x > path.end.x + r ) {
      location.x = path.start.x - r ;
      location.y = path.start.y + (location.y - path.end.y) ;
    }
    
  }
  
  void bordersForMul(MulPointsPath mpath) {
    if( location.x > mpath.getEnd().x + r ) {
      location.x = mpath.getStart().x - r ;
      location.y = mpath.getStart().y + (location.y - mpath.getEnd().y) ;
    }

  }
  
  void bordersForCrowd() {

    if( location.x < -r ) 
      location.x = width + r ;

    if( location.x > width+r )
      location.x = -r ;

    if( location.y < -r ) 
      location.y = height + r ;

    if( location.y > height+r )
      location.y = -r ;

  }
  
}
