
#include <teb_package/visualization/teb_robot_footprint.h>
#include <teb_package/base/obstacle.h>
#include <cmath>

namespace teb
{
  template <int p, int q> 
  void Footprint<p,q>::setPolygonFootPrint()
  {
    
    if(_rob == nullptr)  return;
    
    if(_rob->getTypeDriving() == teb::TypeDriving::FRONT_WHEEL)
    {
       _pos_f.coeffRef(0) = _state[0];
       _pos_f.coeffRef(1) = _state[1];
       _pos_r.coeffRef(0) = _state[0] - _dim[0] * std::cos(_state[2]);
       _pos_r.coeffRef(1) = _state[1] - _dim[0] * std::sin(_state[2]);
    }
	 
    else
    {
       _pos_f.coeffRef(0) = _state[0] + _dim[0] * std::cos(_state[2]);
       _pos_f.coeffRef(1) = _state[1] + _dim[0] * std::sin(_state[2]);
       _pos_r.coeffRef(0) = _state[0];
       _pos_r.coeffRef(1) = _state[1];
    }
    
        
    if(_state[2] >= 0 && _state[2] <= M_PI/2)
      {
	 _alpha = M_PI - (M_PI/2 + _state[2]);		 	 
	
	//_pos_f[0] = _state[0] + (_dim[0]/2) * std::cos(_state[2]);   _pos_f[1] = _state[1] + (_dim[0]/2) * std::sin(_state[2]); 
	//_pos_r[0] = _state[0] - (_dim[0]/2) * std::cos(_state[2]);   _pos_r[1] = _state[1] - (_dim[0]/2) * std::sin(_state[2]);
	    	
	_points.front()[0]  = _pos_f[0] - (_dim[1]/2) * std::cos(_alpha);  _points.front()[1]  = _pos_f[1] + (_dim[1]/2) * std::sin(_alpha); //! Upper left
	_points.at(1)[0]    = _pos_f[0] + (_dim[1]/2) * std::cos(_alpha);  _points.at(1)[1]    = _pos_f[1] - (_dim[1]/2) * std::sin(_alpha); //! Upper right
	_points.at(2)[0]    = _pos_r[0] + (_dim[1]/2) * std::cos(_alpha);  _points.at(2)[1]    = _pos_r[1] - (_dim[1]/2) * std::sin(_alpha); //! Lower right
	_points.back()[0]   = _pos_r[0] - (_dim[1]/2) * std::cos(_alpha);  _points.back()[1]   = _pos_r[1] + (_dim[1]/2) * std::sin(_alpha); //! Lower left                 
	
      }
      
      else if(_state[2] > M_PI/2 && _state[2] <= M_PI)
      {
	_alpha = M_PI/2 - ( M_PI - _state[2]);      		
	
	//_pos_f[0] = _state[0] - (_dim[0]/2) * std::cos(M_PI - _state[2]);  _pos_f[1] = _state[1] + (_dim[0]/2) * std::sin(M_PI - _state[2]);
	//_pos_r[0] = _state[0] + (_dim[0]/2) * std::cos(M_PI - _state[2]);  _pos_r[1] = _state[1] - (_dim[0]/2) * std::sin(M_PI - _state[2]);    
		
	_points.front()[0]  = _pos_f[0] - (_dim[1]/2) * std::cos(_alpha);  _points.front()[1]  = _pos_f[1] - (_dim[1]/2) * std::sin(_alpha);  //! Upper left
	_points.at(1)[0]    = _pos_f[0] + (_dim[1]/2) * std::cos(_alpha);  _points.at(1)[1]    = _pos_f[1] + (_dim[1]/2) * std::sin(_alpha);  //! Upper right
	_points.at(2)[0]    = _pos_r[0] + (_dim[1]/2) * std::cos(_alpha);  _points.at(2)[1]    = _pos_r[1] + (_dim[1]/2) * std::sin(_alpha);  //! Lower right
	_points.back()[0]   = _pos_r[0] - (_dim[1]/2) * std::cos(_alpha);  _points.back()[1]   = _pos_r[1] - (_dim[1]/2) * std::sin(_alpha);  //! Lower left     
		    
      }
      
      else if(_state[2] < -M_PI/2 && _state[2] > -M_PI)
      {
	 _alpha = M_PI - (M_PI/2 + _state[2]);	 	 
	
	//_pos_f[0] = _state[0] + (_dim[0]/2) * std::cos(_state[2]);  _pos_f[1] = _state[1] + (_dim[0]/2) * std::sin(_state[2]);
	//_pos_r[0] = _state[0] - (_dim[0]/2) * std::cos(_state[2]);  _pos_r[1] = _state[1] - (_dim[0]/2) * std::sin(_state[2]);            
		
	_points.front()[0]  = _pos_f[0] + (_dim[1]/2) * std::cos(_alpha);  _points.front()[1]  = _pos_f[1] - (_dim[1]/2) * std::sin(_alpha);  //! Upper left
	_points.at(1)[0]    = _pos_f[0] - (_dim[1]/2) * std::cos(_alpha);  _points.at(1)[1]    = _pos_f[1] + (_dim[1]/2) * std::sin(_alpha);  //! Upper right
	_points.at(2)[0]    = _pos_r[0] - (_dim[1]/2) * std::cos(_alpha);  _points.at(2)[1]    = _pos_r[1] + (_dim[1]/2) * std::sin(_alpha);  //! Lower right
	_points.back()[0]   = _pos_r[0] + (_dim[1]/2) * std::cos(_alpha);  _points.back()[1]   = _pos_r[1] - (_dim[1]/2) * std::sin(_alpha);  //! Lower left 
	      
      }

      else  //! angle is -pi/2 < angle < 0
      {
	_alpha = M_PI/2 + _state[2];
	
	//_pos_f[0] = _state[0] + (_dim[0]/2) * std::cos(_state[2]);  _pos_f[1] = _state[1] + (_dim[0]/2) * std::sin(_state[2]);
	//_pos_r[0] = _state[0] - (_dim[0]/2) * std::cos(_state[2]);  _pos_r[1] = _state[1] - (_dim[0]/2) * std::sin(_state[2]);            
	    	
	_points.front()[0]  = _pos_f[0] + (_dim[1]/2) * std::cos(_alpha);  _points.front()[1]  = _pos_f[1] + (_dim[1]/2) * std::sin(_alpha);  //! Upper left
	_points.at(1)[0]    = _pos_f[0] - (_dim[1]/2) * std::cos(_alpha);  _points.at(1)[1]    = _pos_f[1] - (_dim[1]/2) * std::sin(_alpha);  //! Upper right
	_points.at(2)[0]    = _pos_r[0] - (_dim[1]/2) * std::cos(_alpha);  _points.at(2)[1]    = _pos_r[1] - (_dim[1]/2) * std::sin(_alpha);  //! Lower right
	_points.back()[0]   = _pos_r[0] + (_dim[1]/2) * std::cos(_alpha);  _points.back()[1]   = _pos_r[1] + (_dim[1]/2) * std::sin(_alpha);  //! Lower left               
	
      }            
   }
   
   template <int p, int q>
   Eigen::Vector3d RobotFrame<p,q>::updateCoordinateTrailer(const Eigen::Ref<const Eigen::VectorXd>& state, unsigned int  n_trailer)
   {      
      Eigen::Vector2d pos_r;      
      Eigen::Vector3d pos;      
      
      pos_r = _body.at(n_trailer-2)->getPos_r();
      
      if(state[2+n_trailer] >= 0)
      {
	 pos[0] = pos_r[0] - (_towbar + _body.at(n_trailer-2)->getDim()[0]/2) * std::cos(state[2+n_trailer]);
         pos[1] = pos_r[1] - (_towbar + _body.at(n_trailer-2)->getDim()[0]/2) * std::sin(state[2+n_trailer]);
      }
      
      else
      {
	 pos[0] = pos_r[0] + (_towbar + _body.at(n_trailer-1)->getDim()[0]/2) * std::cos(state[2+n_trailer]);
         pos[1] = pos_r[1] - (_towbar + _body.at(n_trailer-1)->getDim()[0]/2) * std::sin(state[2+n_trailer]);
      }
		           
      pos[2] = state[2+n_trailer];            
      
      return pos;
  }
  
  template <int p, int q>
  Eigen::Vector3d RobotFrame<p,q>::updateCoordinateCar(const Eigen::Ref<const Eigen::VectorXd>& state)
  {
    Eigen::Vector3d pos;  
    
    for(unsigned int i=0;i<3; ++i)  pos[i] = state[i];
    
    return pos;
    
  }
                  
    template <int p, int q>              
    void RobotPlot<p,q>::plotRobot(const SimResults::TimeSeries& ts, const Eigen::Ref<const Eigen::Matrix2d>& ini_final_state)
    {
      Eigen::MatrixXd data(ts.states.rows()+ts.controls.rows()+1, ts.states.cols()); // +1 for time
      data.block(0,0,ts.states.rows(),ts.states.cols()) = ts.states;
      if ( ts.states.cols() == ts.controls.cols() )
      {
	data.block(ts.states.rows(),0,ts.controls.rows(),ts.controls.cols()) = ts.controls;
      }
      else if ( ts.states.cols()-1 == ts.controls.cols() )
      {
	data.block(ts.states.rows(),0,ts.controls.rows(),ts.controls.cols()) = ts.controls;
	data.block(ts.states.rows(),ts.controls.cols(),ts.controls.rows(),1).setZero();
      }
      else
      {
	PRINT_INFO("plotRobot(): TimeSeries object must contain state and control sequences of similar size (n=m, or m=n-1)");
	return;
      }
      if (ts.time.rows() != ts.states.cols())
      {
	PRINT_INFO("plotRobot(): TimeSeries object must contain state and time sequences of identical size (n=m)");
      }
      data.bottomRows(1) = ts.time.transpose();
      plotRobot(data, ini_final_state);      
      
      //PRINT_INFO("Results:\n");
      //std::cout << "Results:" << data << std::endl;
    }
    
    template <int p, int q>
    void RobotPlot<p,q>::plotRobot(const Eigen::Ref<Eigen::MatrixXd>& data)
    {
       Eigen::Matrix2d mat_temp;
       mat_temp.setZero();
       plotRobot(data, mat_temp);
    }
                  
    
    template <int p, int q>
    void RobotPlot<p,q>::plotRobot(const Eigen::Ref<Eigen::MatrixXd>& data, const Eigen::Ref<const Eigen::Matrix2d>& ini_final_state)
    {
      Eigen::Vector4d vec_points;// = Eigen::Vector4d(-20,20-30,30);      
      Eigen::VectorXd pos_idx(4);       
      unsigned int idx_wheel;
      //const PointPolygon* points;                   
      
        if(_plotter)
	{
	  //! Set the robot footprint and wheels parameters...
	  
	  _plotter->setMultiplot();
	  _plotter->setOptionsPlot("gray",0);
	  
	  //! Get the minimum and maximum state values (x and y) 
	 /* vec_points[0] = data.row(0)[0];                //! xmin
	  vec_points[1] = data.row(0)[data.cols()-1];    //! xmax
	  if(vec_points[0] > vec_points[1])  std::swap(vec_points[0], vec_points[1]);
	  
	  vec_points[2] = data.row(1)[0];                //! ymin
	  vec_points[3] = data.row(1)[data.cols()-1];    //! ymax
	  if(vec_points[2] > vec_points[3])  std::swap(vec_points[2], vec_points[3]); */
	 
	 vec_points[0] = -7;	 
	 vec_points[1] = 7;
	 vec_points[2] = -7;
	 vec_points[3] = 7;
	  
	  _plotter->setPlotRange(vec_points);
	  _plotter->plot(data.row(0).head(data.cols()).transpose(), data.row(1).head(data.cols()).transpose(),"X-Y Trajectory","x [m]", "y [m]");  //! Plot the trajectory
	  
	  pos_idx = _plotter->getRobotPosOnTrajectory(data);     
	  
	  plotInitialFinalState(ini_final_state);
	  
	  //############################################################################################
	  //! Plot the obstacles if they are defined of beforehand
	  if(_control_system->getObstacles().size() != 0)
	  {
	    switch(_option)
	    {
	      case OptionPlot::PLOT_SINGLE_OBSTACLES:   plotObstacles();  break;
	      
	      case OptionPlot::PLOT_OBSTACLES_WITH_BORDER:
		
		        plotObstacles();
			for(unsigned int i=0;i<_cluster->getCluster().size(); ++i)
			    _plotter->plotPolygon(_cluster->getCluster().at(i)->getCornerPolygon(), false, "transparent");
	      break;
	      
	      case OptionPlot::PLOT_ONLY_BORDER:
		
		       for(unsigned int i=0;i<_cluster->getCluster().size(); ++i)
                          _plotter->plotPolygon(_cluster->getCluster().at(i)->getCornerPolygon(), false, "solid");
              break;		       
	    }	    	    	   
	  }
	      	  
	  //############################################################################################
	  
	   for(unsigned int i=0; i<pos_idx.size(); i++)
	    {		
	      idx_wheel = 0;
	      
	      for(unsigned int car=1; car<=_robot->getNumberOfCars(); ++car)
	      {
		  //! The body ...
		  drawBody(data.col(pos_idx[i]), car);  //! The index for the first car is 0
		  
		  //! The wheels....		  
		  _plotter->setOptionsPlot("black",0);	 		  
		  drawWheels(data.col(pos_idx[i]), &idx_wheel, car);	   		  		  
	      }	
	      
	      //! Draw the towbar..
	      drawTowBar(_robot->getNumberOfCars());
	    }
	    
	    _plotter->unsetMultiplot();
	}
	
	else  PRINT_INFO("plotRobot() - Cannot plot, since no TebPlotter object assigned");
    }
    
    template <int p, int q>
    void RobotPlot<p,q>::drawBody(const Eigen::Ref<const Eigen::VectorXd>& state, unsigned int car)
    {
       Eigen::Vector3d vec;		  
		  
       //! Se pinta la base del robot 
		  
	if(car >= 2)      
	  vec = _robot->updateCoordinateTrailer(state,car);
	
	else
	  vec = _robot->updateCoordinateCar(state);
	 		  
	 _robot->getReferenceToBody(car-1)->updateCoordinateFootprint(vec);
	
	  if(_robot->_typeRobot == RobotFrame<p,q>::TypeRobot::CAR_LIKE ||_robot->_typeRobot == RobotFrame<p,q>::TypeRobot::TRUCK_AND_TRAILER)
	  {
	     _robot->getReferenceToBody(car-1)->setPolygonFootPrint();	      
	     _plotter->plotPolygon(_robot->getReferenceToBody(car-1)->getPointpolygon(), true, "solid");  //Falta agregar opciones de diferente color     
	  }
  	
          else
	  {
	    _plotter->plotCircle(_robot->getReferenceToBody(car-1)->getCircleParameters(), "gray"); 
	  }
    }
    
    template <int p, int q>
    void RobotPlot<p,q>::drawWheels(const Eigen::Ref<const Eigen::VectorXd>& state, unsigned int* idx_wheel, unsigned int car)
    {
       Eigen::Vector3d vec;		  
       Eigen::VectorXd new_state = state;
		  
       //! Se pinta la base del robot 
		  
	if(car >= 2)
	{	  
	  vec = _robot->updateCoordinateTrailer(state,car);
	  new_state[0] = vec[0];
	  new_state[1] = vec[1];
	}
	
	else
	  vec = _robot->updateCoordinateCar(state);
		  
       for(TypeWheel type = ((car == 1) && (_robot->_typeRobot == RobotFrame<p,q>::TypeRobot::CAR_LIKE)) ? TypeWheel::FRONT_LEFT : TypeWheel::REAR_LEFT;
	type != TypeWheel::DUMMY;
	type = static_cast<TypeWheel>(static_cast<int>(type) + 1))	
	 {
	    if(_robot->_typeRobot == RobotFrame<p,q>::TypeRobot::DIFFERENTIAL_DRIVE)
	      _robot->getReferenceTowheel(*idx_wheel)->updateWheelPosition(new_state, type);
	    
	    else
	       _robot->getReferenceTowheel(*idx_wheel)->updateWheelPosition(new_state, type,car);
	    
	    _robot->getReferenceTowheel(*idx_wheel)->setPolygonFootPrint();
	    _plotter->plotPolygon(_robot->getReferenceTowheel(*idx_wheel)->getPointpolygon(), false, "solid");
		      
	    (*idx_wheel)++;
	 } 
    }
    
    template <int p, int q>
    void RobotPlot<p,q>::drawTowBar(unsigned int number_of_cars)
    {
      PointPolygon points; 
      int j = 0;      
      
      if(number_of_cars == 1) return;
      
      for(unsigned int i=0;i< number_of_cars-1; ++i)
      {
	 points.resize(j+2);
	 points.at(j)   = _robot->getReferenceToBody(i)->getPos_r();
	 points.at(j+1) = _robot->getReferenceToBody(i+1)->getPos_f();	 
	 j=j+2;	 
      }
                    
       _plotter->plotMultiLines(points);
    }
    
    template <int p, int q>
    void RobotPlot<p,q>::plotObstacles()
    {
      PointCircle points_obstacles;
      
      if(_control_system == nullptr)
       {
	  PRINT_INFO("No Obstacles to show..");
	  return;
       }
       
       for(unsigned int i=0; i<_control_system->getObstacles().size(); ++i)
       {
	  points_obstacles.resize(i+1);
	  points_obstacles.at(i)[0] = _control_system->getObstacles().at(i)->getX();
	  points_obstacles.at(i)[1] = _control_system->getObstacles().at(i)->getY();
	  points_obstacles.at(i)[2] = _control_system->getObstacles().at(i)->getRadiusObstacle();
       }
                      
       _plotter->plotFigureObstacle(points_obstacles);
    }
    
    template <int p, int q>
    void RobotPlot<p,q>::plotInitialFinalState(const Eigen::Ref<const Eigen::Matrix2d>& ini_final_state)
    {
       PointCircle points_states;
       std::vector<std::string> strings;
       strings.push_back("initial");
       strings.push_back("final");
       
       points_states.resize(2);
      
       if(_control_system == nullptr)
       {
	  PRINT_INFO("No Obstacles to show..");
	  return;
       }
       
       for(unsigned int i=0;i<2; ++i)
       {
	  points_states.at(i).head(2) = ini_final_state.col(i);
          points_states.at(i)[2] = 0.1; 
	  	  
	  _plotter->plotCircle(points_states.at(i), (strings.at(i).compare("initial")) == 0 ? "blue" : "green");
	  	  
       }                     
    }
}


