
/* 2016.12.01 */

class Path {
  
  PVector start ;
  PVector end ;
 
  float radius ;
 
  Path() {
    radius = 20 ;
    start = new PVector(0     , height) ;
    end   = new PVector(width , height*2/3) ; 
  } 
  
  void display() {
    
    // thick line of path
    strokeWeight( radius * 2 ) ;
    stroke(0 , 100) ;
    line(start.x , start.y , end.x , end.y) ;
    
    // thin line of path
    strokeWeight(1) ;
    stroke(0);
    line(start.x , start.y , end.x , end.y) ;
  }
  
}
