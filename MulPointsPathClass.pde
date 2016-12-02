
/* 2016.12.01 pm 23:52 */

class MulPointsPath {
  
  ArrayList<PVector> points ;
  
  float radius ;
  
  MulPointsPath() {
    radius = 20 ;
    points = new ArrayList<PVector>() ;
  }
  
  void addPoint(PVector point) {
    points.add( point );
  }
  
  PVector getStart() {
    return points.get(0) ;
  }  
  
  PVector getEnd() {
    return points.get(points.size()-1) ;
  }
  
  void display() {
    
    // thick line of path
    stroke(175);
    strokeWeight(radius * 2);
    noFill();
    beginShape();
    for(PVector v : points) {
      vertex(v.x , v.y);
    }
    endShape();
    
    // thin line of path
    stroke(0);
    strokeWeight(1);
    noFill();
    beginShape();
    for(PVector v : points) {
      vertex(v.x , v.y);
    }
    endShape();
    
  }
  
}
