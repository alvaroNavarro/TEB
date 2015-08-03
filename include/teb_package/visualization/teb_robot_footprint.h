
#ifndef __teb_package__teb_robot_footprint__
#define __teb_package__teb_robot_footprint__

#include <teb_package/base/typedefs.h>
#include <teb_package/visualization/teb_plotter.h>
#include <teb_package/base/teb_controller.h>
#include <teb_package/simulation/simulator.h>
#include <teb_package/base/system_dynamics.h>

#include <cmath>

namespace teb
{
   template <int p, int q>
   class Footprint 
   {
    public:
  	    using Vector2d     = typename Eigen::Matrix<double,2,1>;
            using PointPolygon = typename  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;	    	
  
            Footprint(){};
    
            void setPolygonFootPrint();	    	    
	    void updateCoordinateFootprint(const Eigen::Ref<Eigen::Vector3d>& state) 
	    { 
	      _state = state; 
	      _state[2] = norm_angle(_state[2]);
	    }
	    
	    const Eigen::Vector2d& getPos_r() { return(_pos_r); };
	    const Eigen::Vector2d& getPos_f() { return(_pos_f); };
	    
	    void updateDim(const Eigen::Ref<const Eigen::Vector2d>& dim) { _dim = dim; };	    
	    const Eigen::Vector2d& getDim() const { return(_dim); };
	    const PointPolygon& getPointpolygon() const { return(_points); };
	    
	    //! Call this function if your robot is of differential drive type
	    Eigen::Vector3d getCircleParameters() 
	    {
	        _circle_parameters.coeffRef(0) = _state.coeffRef(0);
		_circle_parameters.coeffRef(1) = _state.coeffRef(1);
		_circle_parameters.coeffRef(2) = _dim.coeffRef(0);
		
		return _circle_parameters;
	    }
	    
	    void setRobot(SystemDynamics<p, q>* robot) {_rob = robot; }
    
    protected:

        PointPolygon _points = PointPolygon(4);
	Eigen::Vector3d _circle_parameters;
        Eigen::Vector2d _pos_f;      //! Projection in the front side of the robot
        Eigen::Vector2d _pos_r;      //! Projection in the rear side of the robot
        
        double _alpha;	
		
	Eigen::Vector3d _state;	      //! x , y , angle
	Eigen::Vector2d _dim;         //! length, width	for the robot of type car like and truck and trailer		
	
	SystemDynamics<p, q>* _rob;
	
   public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
   };  
   
    enum class TypeWheel
       {
	 FRONT_LEFT, 
	 FRONT_RIGHT,
	 REAR_LEFT,
	 REAR_RIGHT,
	 DUMMY
      };
      
    
   template <int p, int q>
    class Wheelprint : public Footprint<p,q>
    {
    public:            
            
            
         //Wheelprint() : Footprint<p,q>() {}	 	    	 
	 	 
	 void setTypeWheel(TypeWheel type){ _type = type;};
	 
	 void getReferenceBody(Footprint<p,q>* body) { _robBody = body; };
	 
	 void updateDim()
	 {
	    Eigen::Vector2d dim_temp;
	    dim_temp[0] = _length_wheel;
	    dim_temp[1] = _width_wheel;
	    
	    Footprint<p,q>::updateDim(dim_temp);
	 }
	 
	 void updateWheelPosition(const Eigen::Ref<const Eigen::VectorXd>& state, TypeWheel type)
	 {
	   double radius = Footprint<p,q>::getDim()[0];
	   
	   _state_temp[2] = state[2]; 
	   
	   if(state[2] >= 0 && state[2] <= M_PI)
	    {
	       switch(type)
	       {
		  case  TypeWheel::REAR_LEFT:
		    
		    _state_temp[0] = state[0] - (radius + _dis_body_wheel) * std::cos(M_PI/2 + /*- _alpha*/ - state[2]);
		    _state_temp[1] = state[1] + (radius + _dis_body_wheel) * std::sin(M_PI/2 + /*- _alpha*/ - state[2]);
		    		    		    
		  break;
		  
		  case  TypeWheel::REAR_RIGHT:
		    
		    _state_temp[0] = state[0] + (radius + _dis_body_wheel) * std::cos(M_PI/2 /*- _alpha*/ - state[2]);
		    _state_temp[1] = state[1] - (radius + _dis_body_wheel) * std::sin(M_PI/2 /*- _alpha*/ - state[2]);
		    
		  break;  
		  
		  case TypeWheel::FRONT_LEFT:
		  case TypeWheel::FRONT_RIGHT:
		  case TypeWheel::DUMMY:
		    
		    PRINT_INFO("Do Nothing");
		    
		  break;  
	       }
	    }
	    
	    else if(state[2] >= -M_PI && state[2] < 0)
	    {
	       switch(type)
	       {
		  case  TypeWheel::REAR_LEFT:
		    
		    _state_temp[0] = state[0] - (radius + _dis_body_wheel) * std::cos(state[2] - M_PI/2 /*- _alpha*/);
		    _state_temp[1] = state[1] + (radius + _dis_body_wheel) * std::sin(state[2] - M_PI/2 /*- _alpha*/);
		    		    		    
		  break;
		  
		  case  TypeWheel::REAR_RIGHT:
		    
		    _state_temp[0] = state[0] + (radius + _dis_body_wheel) * std::cos(state[2] - M_PI/2 /*- _alpha*/);
		    _state_temp[1] = state[1] - (radius + _dis_body_wheel) * std::sin(state[2] - M_PI/2 /*- _alpha*/);
		    
		  break;
		  
		  case TypeWheel::FRONT_LEFT:
		  case TypeWheel::FRONT_RIGHT:
		  case TypeWheel::DUMMY:
		    
		    PRINT_INFO("Do Nothing");
		    
		  break;  
	       }
	    }
	    
	     Footprint<p,q>::updateCoordinateFootprint(_state_temp); 
	 }
	 
	 void updateWheelPosition(const Eigen::Ref<const Eigen::VectorXd>& state, TypeWheel type, unsigned int car)
	 {
	  // _type = static_cast<TypeWheel>(type); 
	   
	   
	    if(state[2] >= 0 && state[2] <= M_PI/2)
	    {
	      
	      switch(type)
	      {
		 case TypeWheel::FRONT_LEFT:
		
		    _state_temp[0] = state[0] + _distance_mag * std::cos(state[2] + _distance_angle);
		    _state_temp[1] = state[1] + _distance_mag * std::sin(state[2] + _distance_angle);
		    _state_temp[2] = state[2] + state[3];
		   
		break;
		
		case TypeWheel::FRONT_RIGHT:
		  
		    _state_temp[0] = state[0] + _distance_mag * std::cos(state[2] - _distance_angle);
		    _state_temp[1] = state[1] + _distance_mag * std::sin(state[2] - _distance_angle);
		    _state_temp[2] = state[2] + state[3];
		    
		  break;
		    
		case TypeWheel::REAR_LEFT:
		  
		    _state_temp[0] = state[0] - _distance_mag * std::cos(state[2] - _distance_angle);
		    _state_temp[1] = state[1] - _distance_mag * std::sin(state[2] - _distance_angle);
		    //_state_temp[2] = state[2];
		    _state_temp[2] = state[(car == 1) ? 2: car+2];

		  break;	  
		  
		case TypeWheel::REAR_RIGHT:
		  
		    _state_temp[0] = state[0] - _distance_mag * std::cos(state[2] + _distance_angle);
		    _state_temp[1] = state[1] - _distance_mag * std::sin(state[2] + _distance_angle);
		    _state_temp[2] = state[(car == 1) ? 2: car+2];
		  
		  break;
		  
		default:
		    PRINT_DEBUG("Nothing special");
	      }
	      
	    }
	    
	    else if(state[2] > M_PI/2 && state[2] <= M_PI)
	    {
	      	switch(type)
		{
		  case TypeWheel::FRONT_LEFT:
		  
		      _state_temp[0] = state[0] - _distance_mag * std::cos(M_PI - ( state[2] + _distance_angle));
		      _state_temp[1] = state[1] + _distance_mag * std::sin(M_PI - ( state[2] + _distance_angle));
		      _state_temp[2] = state[2] + state[3];
		    
		  break;
		  
		  case TypeWheel::FRONT_RIGHT:
		    
		      _state_temp[0] = state[0] - _distance_mag * std::cos(M_PI - ( state[2] - _distance_angle));
		      _state_temp[1] = state[1] + _distance_mag * std::sin(M_PI - ( state[2] - _distance_angle));
		      _state_temp[2] = state[2] + state[3];
		      
		    break;
		      
		  case TypeWheel::REAR_LEFT:
		    
		      _state_temp[0] = state[0] + _distance_mag * std::cos(M_PI - state[2] + _distance_angle);
		      _state_temp[1] = state[1] - _distance_mag * std::sin(M_PI - state[2] + _distance_angle);
		      _state_temp[2] = state[(car == 1) ? 2: car+2];

		    break;	  
		    
		  case TypeWheel::REAR_RIGHT:
		    
		      _state_temp[0] = state[0] + _distance_mag * std::cos((M_PI - state[2]) - _distance_angle );
		      _state_temp[1] = state[1] - _distance_mag * std::sin((M_PI - state[2]) - _distance_angle );
		      _state_temp[2] = state[(car == 1) ? 2: car+2];
		    
		    break;
		    
		  default:
		      PRINT_DEBUG("Nothing special");
		}
	    }
	    
	    else if(state[2] <= -M_PI/2 && state[2] >= -M_PI)
	    {
	          switch(type)
		  {
		    case TypeWheel::FRONT_LEFT:
		    
			_state_temp[0] = state[0] - _distance_mag * std::cos(M_PI + state[2] + _distance_angle);
			_state_temp[1] = state[1] - _distance_mag * std::sin(M_PI + state[2] + _distance_angle);
			_state_temp[2] = state[2] + state[3];
		      
		    break;
		    
		    case TypeWheel::FRONT_RIGHT:
		      
			_state_temp[0] = state[0] - _distance_mag * std::cos(M_PI + (state[2] - _distance_angle));
			_state_temp[1] = state[1] - _distance_mag * std::sin(M_PI + (state[2] - _distance_angle));
			_state_temp[2] = state[2] + state[3];
			
		      break;
			
		    case TypeWheel::REAR_LEFT:
		      
			_state_temp[0] = state[0] + _distance_mag * std::cos((M_PI + state[2]) - _distance_angle);
			_state_temp[1] = state[1] + _distance_mag * std::sin((M_PI + state[2]) - _distance_angle);
			_state_temp[2] = state[(car == 1) ? 2: car+2];

		      break;	  
		      
		    case TypeWheel::REAR_RIGHT:
		      
			_state_temp[0] = state[0] + _distance_mag * std::cos((M_PI + state[2]) + _distance_angle);
			_state_temp[1] = state[1] + _distance_mag * std::sin((M_PI + state[2]) + _distance_angle);
			_state_temp[2] = state[(car == 1) ? 2: car+2];
		      
		      break;
		      
		    default:
			PRINT_DEBUG("Nothing special");
		  }
	    }

	    else  //! angle is -pi/2 < angle < 0
	    {
	        switch(type)
		  {
		    case TypeWheel::FRONT_LEFT:
		    
			_state_temp[0] = state[0] + _distance_mag * std::cos(state[2] + _distance_angle);
			_state_temp[1] = state[1] + _distance_mag * std::sin(state[2] + _distance_angle);
			_state_temp[2] = state[2] + state[3];
		      
		    break;
		    
		    case TypeWheel::FRONT_RIGHT:
		      
			_state_temp[0] = state[0] + _distance_mag * std::cos(state[2] - _distance_angle);
			_state_temp[1] = state[1] + _distance_mag * std::sin(state[2] - _distance_angle);
			_state_temp[2] = state[2] + state[3];
			
		      break;
			
		    case TypeWheel::REAR_LEFT:
		      
			_state_temp[0] = state[0] - _distance_mag * std::cos(_distance_angle - state[2]);
			_state_temp[1] = state[1] + _distance_mag * std::sin(_distance_angle - state[2]);
			_state_temp[2] = state[(car == 1) ? 2: car+2];

		      break;	  
		      
		    case TypeWheel::REAR_RIGHT:
		      
			_state_temp[0] = state[0] - _distance_mag * std::cos(_distance_angle*(-1) - state[2]);
			_state_temp[1] = state[1] + _distance_mag * std::sin(_distance_angle*(-1) - state[2]);
			_state_temp[2] = state[(car == 1) ? 2: car+2];
		      
		      break;
		      
		    default:
			PRINT_DEBUG("Nothing special");
		  }
	    } 	   	   	   		    	    
	    
	    Footprint<p,q>::updateCoordinateFootprint(_state_temp); 
	 }
	 
	 void setAlphaDiffDrive() { _alpha = std::atan2(_length_wheel/2, Footprint<p,q>::getDim()[0] + _dis_body_wheel); }	 
	 
    protected:               	          
	 
	 TypeWheel _type;
	 Eigen::Vector3d _state_temp;
	 double _length_wheel = 0.6;
	 double _width_wheel  = 0.2;
	 
	 double _distance_mag   = 0.5;       //! Values for car like robot and truck and trailer
	 double _distance_angle = M_PI/4;
	 
	 //******************************     -> Just for differential drive robot
	 double _alpha;                       //! _alpha is the angle between center of robot and the front side of the wheel
	                                      //!  This angle is computed as follows..
	                                      //!
	                                      //!  _alpha = atan2(lengh_of_wheel/2 , robot radius + distance_between_robot_body_and_wheel)                  
	                                      
         double _dis_body_wheel = 0.8;	                                      	 
	 
	 Footprint<p,q> *_robBody = nullptr;    //! Only to have access to the dimension of the body
    };
    
    
    //! This is the derivated class to plot the box that corresponds to the robot body and the wheels..
    template <int p, int q>
    class RobotFrame 
    {
    public:                             
      
      enum class TypeRobot
      {
	DIFFERENTIAL_DRIVE,
	CAR_LIKE,
	TRUCK_AND_TRAILER
      };
      
      
      RobotFrame(SystemDynamics<p,q>& system) : _system(&system)  
      {
	int idx = 0;	
	
	if(_system == nullptr) return;
	
	//_number_of_cars = (number_of_states <= 4) ? 1:number_of_states - 3;  
	_number_of_cars = (_system->DimStates  <= 4) ? 1:_system->DimStates - 3;
	
	//! Check of type of robot according to the number of local degree of freedom
	if(_system->DimStates == 3)       _typeRobot = TypeRobot::DIFFERENTIAL_DRIVE;
	else if(_system->DimStates == 4)  _typeRobot = TypeRobot::CAR_LIKE;
	else 				  _typeRobot = TypeRobot::TRUCK_AND_TRAILER;
	 
	 //! The basic configuration..
	 //! One block and Four wheels depending on the type of robot
	 for(unsigned int car=0; car<_number_of_cars; ++car)
	 {
	   _body.push_back(new Footprint<p,q>);	   
	   //_body.at(car)->updateDim(dimension);
	   
	   _body.at(car)->updateDim(_system->getRobotDimension());
	   _body.at(car)->setRobot(_system);	   	   
	   
	  for(TypeWheel type = (car == 0 && _typeRobot == TypeRobot::CAR_LIKE) ? TypeWheel::FRONT_LEFT : TypeWheel::REAR_LEFT;
	   type != TypeWheel::DUMMY;
	   type = static_cast<TypeWheel>(static_cast<int>(type) + 1))		   
	    {
	      _wheels.push_back(new Wheelprint<p,q>);
	      _wheels.at(idx)->updateDim();	   
	      
	      if(_typeRobot == TypeRobot::DIFFERENTIAL_DRIVE){	         
	         _wheels.at(idx)->setAlphaDiffDrive();
	      }
	      
	      _wheels.at(idx)->setTypeWheel(type);
	      _wheels.at(idx)->getReferenceBody(_body.at(car));
	      _wheels.at(idx)->setRobot(_system);
		idx++;	   
	    }	   
	 }
	
      }
      
      ~RobotFrame()
      {
	 
	 for(unsigned int i=0;i<_body.size(); ++i)   delete _body.at(i);	 	 
	 for(unsigned int i=0;i<_wheels.size(); ++i) delete _wheels.at(i);	   	
	 
      }           
      
      Eigen::Vector3d updateCoordinateTrailer(const Eigen::Ref<const Eigen::VectorXd>& state, unsigned int n_trailer);      
      Eigen::Vector3d updateCoordinateCar(const Eigen::Ref<const Eigen::VectorXd>& state);
      
      void setNumberOfCars(int number_of_cars) { _number_of_cars = number_of_cars; };
      unsigned int  getNumberOfCars() { return(_number_of_cars); };
      
      Footprint<p,q>*  getReferenceToBody(int car) { return(_body.at(car)); };
      Wheelprint<p,q>* getReferenceTowheel(int wheel) { return(_wheels.at(wheel)); };
      
      double getLengthTowBar() { return(_towbar); };
      void setLengthTowBar(double towbar){ _towbar = towbar; };
      
      TypeRobot _typeRobot;
      
    protected:
      
      unsigned int _number_of_cars = 1;
      unsigned int _max_number_of_wheels_truck = 4;
      unsigned int _max_number_of_wheels_trailer = 2;
      
      std::vector<Footprint<p,q>*> _body;     //! Depend on the number of cars the robot has
      std::vector<Wheelprint<p,q>*> _wheels;     //! The size depends on the total number of wheels           
      
      double _towbar = 0.3;
      
      SystemDynamics<p,q>* _system = nullptr;            
      
    };
    
            
    template <int p, int q>
    class RobotPlot 
    {
    public:    
      
      enum class OptionPlot
      {
	 PLOT_SINGLE_OBSTACLES,
	 PLOT_OBSTACLES_WITH_BORDER,
	 PLOT_ONLY_BORDER
      };
      
        using PointPolygon      = typename std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;
        using PointCircle       = typename std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;	
	using ObstacleContainer = typename std::vector<Obstacle>*;
      
        RobotPlot(TebPlotter* plotter = nullptr, RobotFrame<p,q>* robot = nullptr) : _plotter(plotter), _robot(robot) {}	
	
	RobotPlot(TebPlotter* plotter = nullptr, RobotFrame<p,q>* robot = nullptr, TebController<p,q>* rob = nullptr) : _plotter(plotter), _robot(robot), _control_system(rob) {} 	
	
	~RobotPlot() {};		
	
	void setIndivudualObstacleBool(bool individual){ _indivudual_obstacles = individual;}
	void setCluster(ClusterObstacle* cluster) {_cluster = cluster;}
	
	void plotRobot(const Eigen::Ref<Eigen::MatrixXd>& data, const Eigen::Ref<const Eigen::Matrix2d>& ini_final_state);
	void plotRobot(const SimResults::TimeSeries& ts, const Eigen::Ref<const Eigen::Matrix2d>& ini_final_state);
	void plotRobot(const Eigen::Ref<Eigen::MatrixXd>& data);
	void plotObstacles();
	void plotInitialFinalState(const Eigen::Ref<const Eigen::Matrix2d>& ini_final_state);
	
	void drawBody(const Eigen::Ref<const Eigen::VectorXd>& state, unsigned int car);
	void drawWheels(const Eigen::Ref<const Eigen::VectorXd>& state, unsigned int* idx_wheel, unsigned int car);
	void drawTowBar(unsigned int number_of_cars);
	
	OptionPlot _option = OptionPlot::PLOT_SINGLE_OBSTACLES;
	
   protected:
        
        TebPlotter* _plotter = nullptr;
	RobotFrame<p,q>* _robot = nullptr;  
        TebController<p,q>* _control_system = nullptr;
	
	bool _indivudual_obstacles = true;
	
	ClusterObstacle* _cluster = nullptr;		
    };
}



#include <teb_package/visualization/teb_robot_footprint.hpp>

#endif   /* (__teb_package__teb_robot_footprint__) */
