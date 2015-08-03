#ifndef __teb_package__obstacle__
#define __teb_package__obstacle__

#include <Eigen/Core>
#include <teb_package/base/typedefs.h>


#include <cmath>
#include <iomanip>

namespace teb
{   
    class Obstacle
    {
	
      public:
	Obstacle(double x, double y, double radius) 
	{ 	  
	  setObstaclePosition(x, y, "initial");
	  setObstaclePosition(x, y, "current");
	  
	  _radius = radius;
	}
	
	Obstacle(const Eigen::Ref<const Eigen::Vector2d>& obs, double radius)
	{
	   _obstacle_posIni = obs;
	   _obstacle_pos    = obs;
	   _radius          = radius; 
	}
				    	
	Eigen::Vector2d getObstaclePosition(std::string string)   const  
	{
	  Eigen::Vector2d obs_pos;
	  
	  if(!string.compare("initial"))
	     obs_pos = getObstacleInitialPosition();
	  
	  else if(!string.compare("final"))
	     obs_pos = getObstacleFinalPosition();
	  
	  else if(!string.compare("current"))
	     obs_pos = getObstacleCurrentPosition();
	  
	  else if(!string.compare("final_fixed"))
	     obs_pos = getObstacleFinalFixedPosition();
	  
	  return(obs_pos); 	  
	}
	
	void setObstacleVelocity(double vel) {_curr_velocity = vel;}
	
	bool setObstacleDeltaX(double dt) 
	{
	  _step = std::abs(_curr_velocity) * dt;
	  
	  if(_step > (_obstacle_posFinalFixed - _obstacle_posIni).norm())
	    return false;	  	  

         return true;	  
	}		
	
	//! This function is to change the obstacle position by using directly the obstacle path already created. Of this way
	//! the change is caried out passing the index as parameter to the function...
	void setObstaclePosition(unsigned int index_array, std::string string)
	{
	   if(index_array > _length_path)  index_array = _length_path;   //! To avoid an invalid memory...
	   
	   if(_obstacle_path.empty())
	   {
	      PRINT_INFO("Do not exist path to move the obstacle..");
	      return;
	   }
	   
	   setObstaclePosition(_obstacle_path.at(index_array), string);
	}
	
	void setObstaclePosition(double x, double y, std::string string)
	{
	   Eigen::Vector2d obs_pos = Eigen::Vector2d(x,y);	   	   
	   setObstaclePosition(obs_pos, string);
	}
	
	void setObstaclePosition(const Eigen::Ref<const Eigen::Vector2d>& obs_pos, std::string string)
	{
	  if(!string.compare("initial"))	  
	  _obstacle_posIni = obs_pos;	  
	  
	  else if(!string.compare("current"))
	    _obstacle_pos = obs_pos;
	  
	  else if(!string.compare("final"))
	    _obstacle_posFinal = obs_pos;
	  
	  else if(!string.compare("final_fixed"))
	    _obstacle_posFinalFixed = obs_pos;
	}
	
	unsigned int getLengthPath() {return (_length_path); }
	
	void initializeObstaclePath()
	{
	   if((_obstacle_posFinalFixed - _obstacle_posIni).norm() < 0.1) return;    //! This is a point not a rect
	   
	   double ang;
	   
	   //! If the path is a straight line in Y axis direction, atan2 -> inf. Hence the angle is pi/2
	   if(( _obstacle_posFinalFixed[0] - _obstacle_posIni[0]) < 0.05)  
	     ang = M_PI/2;
	   else  
	     ang = std::atan2(_obstacle_posFinalFixed[1] - _obstacle_posIni[1], _obstacle_posFinalFixed[0] - _obstacle_posIni[0]);
	   
	   _length_path = (unsigned int)((_obstacle_posFinalFixed - _obstacle_posIni).norm() / _step);	
	   	   
	   Eigen::Vector2d vec_dir = _obstacle_posFinalFixed - _obstacle_posIni;
	   
	   if(vec_dir[0] < 0 && vec_dir[1] > 0)      ang = M_PI + ang;
	   else if(vec_dir[0] < 0 && vec_dir[1] < 0) ang = -M_PI + ang;
	   
	   Eigen::Vector2d point;
	   
	   for(unsigned int i=0; i<_length_path; ++i)
	   {	     
	      point[0] = _obstacle_posIni[0] + (i * _step) * std::cos(ang);     //! Point in X
	      point[1] = _obstacle_posIni[0] + (i * _step) * std::sin(ang);     //! Point in Y
		  
	      _obstacle_path.push_back(point);
	   }
	       
	  /* if((_obstacle_posFinalFixed[0] - _obstacle_posIni[0]) > 0.1)
	   {
	        double m = (_obstacle_posFinalFixed.coeffRef(1) - _obstacle_posIni.coeffRef(1)) / (_obstacle_posFinalFixed.coeffRef(0) - _obstacle_posIni.coeffRef(0)); 
		double b = _obstacle_posIni.coeffRef(1) - (m * _obstacle_posIni.coeffRef(0));
		
		_length_path = (unsigned int)(std::abs(_obstacle_posFinalFixed[0] - _obstacle_posIni[0]) / _step);	   
		
		Eigen::Vector2d point;		
				
		for(unsigned int i=0; i<_length_path; ++i)
		{	     
		  point[0] =   (_obstacle_posIni[0] > _obstacle_posFinalFixed[0]) ? _obstacle_posIni[0] - (i * _step) : (i * _step) + _obstacle_posIni[0];     //! Point in X
		  point[1] = m * point[0] + b;                      //! Point in Y
		  
		  _obstacle_path.push_back(point);
		}   
	   }
	   
	   else
	   {
	       //! The rect is perpendicular to the x axis... 
	     _length_path = (unsigned int)(std::abs(_obstacle_posFinalFixed[1] - _obstacle_posIni[1]) / _step);
	   /  
	     Eigen::Vector2d point = Eigen::Vector2d::Zero();
	    	     	    		     
	     for(unsigned int i=0; i<_length_path; ++i)
	     {	     
	        point[0] = _obstacle_posIni[0];      			//! Point in X
	        point[1] = (_obstacle_posIni[1] > _obstacle_posFinalFixed[1]) ? _obstacle_posIni[1] - (i * _step) : (i * _step) + _obstacle_posIni[1];                  //! Point in Y
		  
	        _obstacle_path.push_back(point);
	     }
	   }	*/
	  	   	   	   
	}		
	
	double getX() {return(_obstacle_pos[0]); };
	double getY() {return(_obstacle_pos[1]); };
	
	void setRadiusObstacle(double radius) {_radius = radius; };
	double getRadiusObstacle() {return(_radius); };	
	double getMaximumRadius() {return(_max_radius); }
	double getMinimunRadius() {return(_min_radius); }
	
      private:
	void setRadiusPointObstacle(double radius) { _radiusPoint = radius; }
	double getRadiusPointObstacle(){ return(_radiusPoint); }
	
	const Eigen::Vector2d& getObstacleCurrentPosition()     const  {return(_obstacle_pos); };
	const Eigen::Vector2d& getObstacleInitialPosition()     const  {return(_obstacle_posIni); };
	const Eigen::Vector2d& getObstacleFinalPosition()       const  {return(_obstacle_posFinal); };
	const Eigen::Vector2d& getObstacleFinalFixedPosition()  const  {return(_obstacle_posFinalFixed); };
		
      protected:
	
	Eigen::Vector2d _obstacle_pos;
	Eigen::Vector2d _obstacle_posIni;
	Eigen::Vector2d _obstacle_posFinal;
	Eigen::Vector2d _obstacle_posFinalFixed;
	
	std::vector<Eigen::Vector2d> _obstacle_path;
	double _step = 0.1;
	unsigned int _length_path;
	
	double _radiusPoint = 0.04;
	double _radius = 1;
	
	double _max_radius = 1.5;
	double _min_radius = 0.1;		

        double _curr_velocity;	
    };  
    
    //! This class define the figure obstacle that represent the polygon on the map...    
    class PolygonObstacle
    {
    public:                       
      
          using ObstacleContainer = typename std::vector<Obstacle*>;
	  using PointPolygon = typename std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;	  	  
	  
 	  PolygonObstacle() { _n_points = 0;}
	  	 
	  void setDt(double step) {_dt = step; }
	  double getDt() {return(_dt);}
	  
	  void insertObstacle(Obstacle* obstacle)
	  {
	     _obstacles.push_back(obstacle); 
	  }
	  
	  void addPointPolygon(const std::array<double, 2>& point)
	  {	     
	     addPointPolygon(Eigen::Map<const Eigen::Vector2d>(point.data()));
	  }
	  
	  void addPointPolygon(const Eigen::Ref<const Eigen::Vector2d>& point)
	  {	     
	    //_points.resize(_n_points + 1);  
	    _points.push_back(point);
	     
	     ++_n_points;
	  }
	  
	  const ObstacleContainer& getObstacleContainer() {return(_obstacles);}
	  
	  const PointPolygon& getCornerPolygon() { return(_points); }
	  
	  void createObstacleFigure()
	  {
	    double side1 = (_points.at(0) - _points.at(1)).norm();
	    double side2 = (_points.at(1) - _points.at(2)).norm();
	    	     
	     //! Now the centroid of each circle is obtained and the obstacle is created...
	     double posx;
	     double posy=0;
	     
	     
	     double initial_radius = 2;
	     double length1 = 0;
	     double length2 = 0;
	     double posy_im1;
	     double posx_im1;
	     
	       
	     while((2*initial_radius) > side1)
		    initial_radius -= 1;
	       
	     //! Set the centroid
	     posx = _points.at(1)[0] + initial_radius;	    
	     posy = _points.at(1)[1] - initial_radius;
	     
	     length1 = 2*initial_radius;
	     length2 = 2*initial_radius;	       	       	     
	       
	     //! Sweep the x axis first....	      
	     do
	     {
	       //! A new obstacle is inserted into the container..
	       insertObstacle(new Obstacle(posx, posy, initial_radius));	       
	       
               //! Sweep the Y axis...
	       do
		{
                      posy_im1 = posy;
		      posy -= (2*initial_radius);		    
		      length1 += (2*initial_radius);    
			
		      while(length1  > side1){
			  posy += 1;
			  length1 -= 1;
		      }
					
		      //! A new obstacle is inserted into the container..
		      if(posy_im1 != posy)
			insertObstacle(new Obstacle(posx, posy, initial_radius)); 
			
		}while((length1 <= side1) && (posy_im1 != posy));
		 		 
		posx_im1 = posx;
		posx += (2*initial_radius);
		length2 += (2*initial_radius); 
		  
		while(length2  > side2){
		   posx -= 1;
		   length2 -= 1;
		}
		
		 posy = _points.at(1)[1] - initial_radius;	       	       		    
	         length1 = 2*initial_radius;
		 
	     }while((length2 <= side2) && (posx_im1 != posx));	                
	  }	  	 
	  
	  //void createObstacleLine(const Eigen::Ref<const Eigen::Vector2d>& p1,const Eigen::Ref<const Eigen::Vector2d>& p2)
	  void createObstacleLine(unsigned int idx_p1, unsigned int idx_p2)
	  {	     
	    Eigen::Vector2d proj_vector;
	    Eigen::Vector2d p_i;
	    
	     if(_points.at(idx_p2).norm() > _points.at(idx_p1).norm()){
	       proj_vector = _points.at(idx_p2) - _points.at(idx_p1);	
	      p_i = _points.at(idx_p1);
	     }
	    
	     else{
	       proj_vector = _points.at(idx_p1) - _points.at(idx_p2);
	       p_i = _points.at(idx_p2);
	     }
	    
	     double dis = (_points.at(idx_p1) - _points.at(idx_p2)).norm();	    	    	    
	     double ang = std::atan2(proj_vector.coeffRef(1), proj_vector.coeffRef(0));
	     	     	     
	     Eigen::Vector2d p_ip1;
	     
	     double step = dis / _dt;
	     
	     for(unsigned int i=0; i< std::floor(step); ++i)
	     {
	         p_ip1.coeffRef(0) = p_i.coeffRef(0) + _dt * std::cos(ang);
		 p_ip1.coeffRef(1) = p_i.coeffRef(1) + _dt * std::sin(ang);
		 
		 //! The obstacle is instaciated and store into the container...
		 insertObstacle(new Obstacle(p_ip1.coeffRef(0), p_ip1.coeffRef(1), 0.015));
		 
		 //! Update the point position
		 p_i = p_ip1;
	     }
	  }
	  
	  void createObstacle()
	  {
	     if(_n_points != 4){
	         PRINT_INFO("The set must to contain 4 points...");
		 return;
	     }
	     
	     createObstacleFigure();
	     _n_points = 0;	    
	  }	  	  
      
    protected:                      
      
      PointPolygon _points;
      ObstacleContainer _obstacles;
      unsigned int _n_points;
      
      unsigned int _number_obstacles_circles = 0;
      
      double _dt = 0.1;
    };
    
    class ClusterObstacle
    {
    public:
          
          using Cluster = typename std::vector<PolygonObstacle*>;
	  
	  ClusterObstacle()
	  {
	    //std::uniform_int_distribution<unsigned int> random_scene(1, _max_scenes); 
	    //_number_scene = random_scene(generator);
	    
	    //insertPolygons();
	  }
	  
	  ~ClusterObstacle()
	  {
	     for(unsigned int i=0; i<_number_cluster; ++i)  delete _cluster.at(i); 
	  }
	  
	  const Cluster& getCluster() {return(_cluster);}
	  
	  void insertPolygons()
	  {
	      switch(_number_scene)
	      {
		case 1:
		case 2:
		  addCluster(new PolygonObstacle);
		break;
		
		case 3:
		case 4:
		  for(unsigned int i=0; i<2; ++i)		
		    addCluster(new PolygonObstacle);		
		break;
		
		case 5:
		  for(unsigned int i=0; i<3; ++i)		
		    addCluster(new PolygonObstacle);		
		break;
		
		case 6:
		  for(unsigned int i=0; i<4; ++i)		
		    addCluster(new PolygonObstacle);		
		break;
		
		case 7:
		  for(unsigned int i=0; i<5; ++i)		
		    addCluster(new PolygonObstacle);		
		break;
		
		case 8:
		  for(unsigned int i=0; i<3; ++i)		
		    addCluster(new PolygonObstacle);		
		break;	
		
		case 9:
		  for(unsigned int i=0; i<3; ++i)		
		    addCluster(new PolygonObstacle);		
		break;
	      } 
	      
	      _number_cluster = getCluster().size();
	  }
	  
	  unsigned int getNumberCluster() {return(_number_cluster);}
	  
	  ClusterObstacle(unsigned int number_scene) { addScene(number_scene);}	  
	  void addScene(unsigned int number_scene)
	  {
	    if(number_scene > _max_scenes)
	       _number_scene = _max_scenes;
	    
	    else
	       _number_scene = number_scene;

            insertPolygons();	    
	  }
	  
	  void generateScene()
	  {
	    if(_cluster.size() == 0)
	    {
	       PRINT_INFO("No cluster defined...");
	       return;
	    }
	    
	    switch(_number_scene)
	     {
	       case 1:  scene_1();  break;
	       case 2:  scene_2();  break;
	       case 3:  scene_3();  break;
	       case 4:  scene_4();  break;
	       case 5:  scene_5();  break;
	       case 6:  scene_6();  break;
	       case 7:  scene_7();  break;
	       case 8:  scene_8();  break;
	       case 9:  scene_9();  break;
	     }
	  }
	  
	  void addCluster(PolygonObstacle* polyObs)
	  {
	    _cluster.push_back(polyObs); 
	  }
	  
	  void scene_1()
	  {
	     _cluster.front()->addPointPolygon({-3,-4});
	     _cluster.front()->addPointPolygon({-3,6});
	     _cluster.front()->addPointPolygon({3,6});
	     _cluster.front()->addPointPolygon({3,-4});
	     
	     _cluster.front()->createObstacle();
	  }
	  
	  void scene_2()
	  {
	     _cluster.front()->addPointPolygon({-5,-3});
	     _cluster.front()->addPointPolygon({-5,3});
	     _cluster.front()->addPointPolygon({5,3});
	     _cluster.front()->addPointPolygon({5,-3});
	     
	     _cluster.front()->createObstacle();
	  }
	  
	  void scene_3()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-5,-2});
	     _cluster.at(0)->addPointPolygon({-5,2});
	     _cluster.at(0)->addPointPolygon({-2,2});
	     _cluster.at(0)->addPointPolygon({-2,-2});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({2,3});
	     _cluster.at(1)->addPointPolygon({2,6});
	     _cluster.at(1)->addPointPolygon({5,6});
	     _cluster.at(1)->addPointPolygon({5,3});
	     
	     _cluster.at(1)->createObstacle();	  
	  }
	  
	  void scene_4()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-5,2});
	     _cluster.at(0)->addPointPolygon({-5,4});
	     _cluster.at(0)->addPointPolygon({0,4});
	     _cluster.at(0)->addPointPolygon({0,2});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({0,-5});
	     _cluster.at(1)->addPointPolygon({0,-3});
	     _cluster.at(1)->addPointPolygon({5,-3});
	     _cluster.at(1)->addPointPolygon({5,-5});
	     
	     _cluster.at(1)->createObstacle();	  
	  }
	  
	  void scene_5()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-6,3});
	     _cluster.at(0)->addPointPolygon({-6,6});
	     _cluster.at(0)->addPointPolygon({-4,6});
	     _cluster.at(0)->addPointPolygon({-4,3});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({-1,-2});
	     _cluster.at(1)->addPointPolygon({-1,2});
	     _cluster.at(1)->addPointPolygon({1,2});
	     _cluster.at(1)->addPointPolygon({1,-2});
	     
	     _cluster.at(1)->createObstacle();
	     
	     //! Polygon 3
	     _cluster.at(2)->addPointPolygon({3,3});
	     _cluster.at(2)->addPointPolygon({3,6});
	     _cluster.at(2)->addPointPolygon({5,6});
	     _cluster.at(2)->addPointPolygon({5,3});
	     
	     _cluster.at(2)->createObstacle();	  
	  }
	  
	  void scene_6()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-4,-6});
	     _cluster.at(0)->addPointPolygon({-4,-3});
	     _cluster.at(0)->addPointPolygon({-2,-3});
	     _cluster.at(0)->addPointPolygon({-2,-6});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({-4,3});
	     _cluster.at(1)->addPointPolygon({-4,6});
	     _cluster.at(1)->addPointPolygon({-2,6});
	     _cluster.at(1)->addPointPolygon({-2,3});
	     
	     _cluster.at(1)->createObstacle();
	     
	     //! Polygon 3
	     _cluster.at(2)->addPointPolygon({2,3});
	     _cluster.at(2)->addPointPolygon({2,6});
	     _cluster.at(2)->addPointPolygon({4,6});
	     _cluster.at(2)->addPointPolygon({4,3});
	     
	     _cluster.at(2)->createObstacle();
	     
	     //! Polygon 4
	     _cluster.at(3)->addPointPolygon({2,-6});
	     _cluster.at(3)->addPointPolygon({2,-3});
	     _cluster.at(3)->addPointPolygon({4,-3});
	     _cluster.at(3)->addPointPolygon({4,-6});
	     
	     _cluster.at(3)->createObstacle();	  
	  }

	  
	  void scene_7()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-6,-5});
	     _cluster.at(0)->addPointPolygon({-6,-2});
	     _cluster.at(0)->addPointPolygon({-4,-2});
	     _cluster.at(0)->addPointPolygon({-4,-5});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({-6,2});
	     _cluster.at(1)->addPointPolygon({-6,4});
	     _cluster.at(1)->addPointPolygon({-4,4});
	     _cluster.at(1)->addPointPolygon({-4,2});
	     
	     _cluster.at(1)->createObstacle();
	     
	     //! Polygon 3
	     _cluster.at(2)->addPointPolygon({-2,-4});
	     _cluster.at(2)->addPointPolygon({-2,2});
	     _cluster.at(2)->addPointPolygon({2,2});
	     _cluster.at(2)->addPointPolygon({2,-4});
	     
	     _cluster.at(2)->createObstacle();
	     
	     //! Polygon 4
	     _cluster.at(3)->addPointPolygon({4,-5});
	     _cluster.at(3)->addPointPolygon({4,-2});
	     _cluster.at(3)->addPointPolygon({6,-2});
	     _cluster.at(3)->addPointPolygon({6,-5});
	     
	     _cluster.at(3)->createObstacle();
	     
	     //! Polygon 5
	     _cluster.at(4)->addPointPolygon({4,2});
	     _cluster.at(4)->addPointPolygon({4,4});
	     _cluster.at(4)->addPointPolygon({6,4});
	     _cluster.at(4)->addPointPolygon({6,2});
	     
	     _cluster.at(4)->createObstacle();	  
	  }
	  
	  void scene_8()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-6,-4});
	     _cluster.at(0)->addPointPolygon({-6,-2});
	     _cluster.at(0)->addPointPolygon({-2,-2});
	     _cluster.at(0)->addPointPolygon({-2,-4});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({2,-4});
	     _cluster.at(1)->addPointPolygon({2,-2});
	     _cluster.at(1)->addPointPolygon({6,-2});
	     _cluster.at(1)->addPointPolygon({6,-4});
	     
	     _cluster.at(1)->createObstacle();
	     
	     //! Polygon 3
	     _cluster.at(2)->addPointPolygon({-4,4});
	     _cluster.at(2)->addPointPolygon({-4,6});
	     _cluster.at(2)->addPointPolygon({4,6});
	     _cluster.at(2)->addPointPolygon({4,4});
	     
	     _cluster.at(2)->createObstacle();	  
	  }
	  
	  void scene_9()
	  {
	     //! Polygon 1
	     _cluster.at(0)->addPointPolygon({-1,-5});
	     _cluster.at(0)->addPointPolygon({-1,-4});
	     _cluster.at(0)->addPointPolygon({0,-4});
	     _cluster.at(0)->addPointPolygon({0,-5});
	     
	     _cluster.at(0)->createObstacle();
	     
	     //! Polygon 2
	     _cluster.at(1)->addPointPolygon({-1,3});
	     _cluster.at(1)->addPointPolygon({-1,7});
	     _cluster.at(1)->addPointPolygon({0,7});
	     _cluster.at(1)->addPointPolygon({0,3});
	     
	     _cluster.at(1)->createObstacle();
	     
	     //! Polygon 3
	     _cluster.at(2)->addPointPolygon({4,-5});
	     _cluster.at(2)->addPointPolygon({4,7});
	     _cluster.at(2)->addPointPolygon({5,7});
	     _cluster.at(2)->addPointPolygon({5,-5});
	     
	     _cluster.at(2)->createObstacle();	  
	  }
	  
	  unsigned int _max_scenes = 9;
	  
    protected:
      
        std::random_device rd;   // non-deterministic generator
        std::default_random_engine generator;
      
        Cluster _cluster;
        unsigned int _number_scene;
	
	unsigned int _number_cluster;
    };

}


#endif /* defined(__teb_package__obstacle__) */
