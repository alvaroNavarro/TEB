#ifndef __teb_package__teb_plotter__
#define __teb_package__teb_plotter__

#ifndef RTW

#include <teb_package/base/typedefs.h>
#include <teb_package/base/teb_controller.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <Eigen/Core>

namespace teb
{
  
/**
 * @brief Customize plots and figures.
 * 
 * This option class can be used to customize figures and plots,
 * if they are supported by the plot function.
 *
 * @ingroup visualization
 * 
 * @todo Extend functionality
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */    
struct PlotOptions
{
    std::string title = ""; //!< Window title
    bool legend = false; //!< Show legend if \c true.
    std::vector<std::string> legend_entries; //!< Vector of strings defining the names for the legend @remark legend should be enabled.
    bool skip_last_value_right_column = false; //! < If \c true, the last value of the right column for each plot will be ignored (use to supress plotting the invalid control for time n)
    std::vector<std::string> ylabels; //!< Vector of strings to define the y labels (Column-Major Order) - leave blank in order to use default names
    
    std::string color = "";   //! Define the color for the line
    int line_style;           //! Define the line style
    bool show_grid;           //! Allow to show the grid on the plot
};



/**
 * @brief Provides useful visualization and plotting functions.
 * 
 * This class provides visualization and plot functions
 * for dynamic systems and the TebController.
 * In addition this class defines plot functions for custom
 * data specified by Eigen::Vector types.
 * 
 * Requirements: This class needs gnuplot installed correctly and
 * it must be determined by cmake first (GNUPLOT_PATH needs to be valid).
 *
 * @ingroup visualization
 * 
 * @bug EPS and PDF export are not working as expected. No text is displayed and after importing to Inskape, the plot is blank (noticed on Ubuntu 14.04).
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */  

class TebPlotter
{
public:        
  
  using Vector2d          = typename Eigen::Matrix<double,2,1>;
  using PointPolygon      = typename  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;
  using PointCircle       = typename std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
  
  /** Supported file formats for figure export */
  enum class FileFormat {PNG, JPEG, SVG, PDF, EPS, TIKZ};
  
  //! Constructor
  TebPlotter();
  //! Destructor
  ~TebPlotter();

  //! Plot a single vector over the range 0 .. n
  void plot1DVector(const Eigen::Ref<const Eigen::VectorXd>& data);
  
  //! Create a common 2D plot
  void plot(const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& y, std::string title="", std::string xlabel="", std::string ylabel="");
    
  //! Plot the sparsity pattern of a given matrix
  void spyMatrix(const Eigen::Ref<const Eigen::MatrixXd>& matrix);
    
  //! Plot TEB states and control inputs
  template <typename StateSeq, typename ControlSeq>
  void plotTEB(const StateSeq& state_seq, ControlSeq& ctrl_seq, double dt);
  
  /**
   * @brief Plot TEB states and control inputs
   * @param teb Reference to a TebController object
   * @tparam Type of the TebController object (deduced by the compiler)
   */
  template <typename TebCtrl>
  void plotTEB(const TebCtrl& teb)
  {
     plotTEB(teb.stateSequence(),teb.controlSequence(), teb.dt().dt());
  };
    
  
  //! Create a two column multiplot with a single line for each plot.
  void plotTwoCol(const Eigen::Ref<const Eigen::VectorXd>& col1_time, const Eigen::Ref<const Eigen::MatrixXd>& col1_data,
                  const Eigen::Ref<const Eigen::VectorXd>& col2_time, const Eigen::Ref<const Eigen::MatrixXd>& col2_data, PlotOptions* options = nullptr);
  
  //! Create a two column multiplot with multiple lines for each plot.
  void plotTwoCol(const std::vector<const Eigen::VectorXd*>& col1_time, const std::vector<const Eigen::MatrixXd*>& col1_data,
                  const std::vector<const Eigen::VectorXd*>& col2_time, const std::vector<const Eigen::MatrixXd*>& col2_data, PlotOptions* options = nullptr);
   
  
    //! Create a one column multiplot with a single line for each plot.
  void plotMulti(const Eigen::Ref<const Eigen::VectorXd>& col1_time, const Eigen::Ref<const Eigen::MatrixXd>& col1_data, PlotOptions* options = nullptr)
  {
    Eigen::Matrix<double,0,1> empty_vec;
    Eigen::Matrix<double,0,0> empty_mat;
    plotTwoCol(col1_time, col1_data, empty_vec, empty_mat, options);
  }
  
  //! Create a one column multiplot with multiple lines for each plot.
  void plotMulti(const std::vector<const Eigen::VectorXd*>& col1_time, const std::vector<const Eigen::MatrixXd*>& col1_data, PlotOptions* options = nullptr)
  {
      std::vector<const Eigen::VectorXd*> empty_vec;
      std::vector<const Eigen::MatrixXd*> empty_mat;
      plotTwoCol(col1_time, col1_data, empty_vec, empty_mat, options);
  }    
  
  //! Plot a 2 dimensional polygon in the 2D plane.
  void plotPolygon(const PointPolygon& data, bool front, std::string string);
  
  //! This function draw a line on the plane
  void plotMultiLines(const PointPolygon& points);
              
  void plotSingleLine(const Eigen::Ref<const Eigen::Vector2d>& point1, const Eigen::Ref<const Eigen::Vector2d>& point2);        
  
  void plotFigureObstacle(const PointCircle& point_obs);
  
  void plotCircle(const Eigen::Ref<const Eigen::Vector3d>& point, std::string str);
  
                
  //! Draw the robot on the plane
  //void plotRobotSequenceTrajectory(const Eigen::Ref<Eigen::MatrixXd>& data);
  
  //! Draw the trajectory of the robot as well as the robot footprint
  //void plotRobotPlusTrajectory(const Eigen::Ref<Eigen::MatrixXd>& data);
  
  void setPlotRange(const Eigen::Ref<Eigen::Vector4d>& vec);    
  
  void setPlotRange();
 
    
  /**
   * @brief Clear current gnuplot window.
   */
  void clearWindow()
  {
    assert(pipe);
    fprintf(pipe, "clear\n");
//     fprintf(pipe, "reset\n");
  }
  
  void setMultiplot()
  {
     assert(pipe);
     fprintf(pipe, "set multiplot \n");
  }
  
  void unsetMultiplot()
  {
     assert(pipe);
     fprintf(pipe, "unset multiplot \n");
     fflush(pipe);
  }
  
  /**
   * @brief Switch gnuplot window or create a new one.
   * @param window_id Switch to gnuplot window with id \c window_id. If it does not exist, create new window.
   * @param reset Reset window settings (recommand after switching from multiplots with extra sizes)
   */
  void switchWindow(int window_id, bool reset=true)
  {
    assert(pipe);
    fprintf(pipe, "@TERM %d persist\n",window_id);
    if (reset) fprintf(pipe, "reset\n"); // Reset size of previous terms
      
    _file_export = false;
  }
  
  /**
   * @brief Close gnuplot window with id \c window_id.
   * @param window_id that should be closed.
   */
  void closeWindow(int window_id = 0)
  {
    assert(pipe);
    fprintf(pipe, "@TERM close %d\n",window_id);
  }
  
  
  /**
   * @brief Change output to file.
   * @param filename Export figure to a file called filename into the current working directory or specify complete path.
   * @param format Fileformat chosen from TebPlotter::FileFormat enum.
   */
  void setOutputToFile(std::string filename, FileFormat format = FileFormat::PNG)
  {
    assert(pipe);
    std::string suffix = "png";
    
    switch (format)
    {
      case FileFormat::SVG:
            fprintf(pipe, "TERM = \"set term svg enhanced\"\n");
            suffix = "svg";
            break;
	
      case FileFormat::PDF:
            #ifdef APPLE
                PRINT_DEBUG("PDF export on OS X currently not supported. Please change output file format.");
            #endif
            #ifdef WIN32
                PRINT_DEBUG_ONCE("Warning, PDF export may not work, if pdfcairo terminal does not exist. Check manually whether output was saved correctly.");
            #endif
            _pdf_flag = true;
            fprintf(pipe, "TERM = \"set term pdfcairo enhanced\"\n");
            suffix = "pdf";
            break;
            
      case FileFormat::EPS:
            fprintf(pipe, "TERM = \"set term postscript eps enhanced\"\n");
            suffix = "eps";
            break;
	
      case FileFormat::TIKZ:
            fprintf(pipe, "TERM = \"set term tikz\"\n");
            suffix = "tikz";
            break;
            
      case FileFormat::JPEG:
            fprintf(pipe, "TERM = \"set term jpeg enhanced\"\n");
            suffix = "jpg";
            break;
            
      default:
            #if defined(WIN32) || defined(APPLE)
                fprintf(pipe, "TERM = \"set term png enhanced\"\n");
            #else
                fprintf(pipe, "TERM = \"set term pngcairo enhanced\"\n");
            #endif
    }
    // apply term
    fprintf(pipe, "@TERM\n");
    //set output to file with given filename
    fprintf(pipe, "set output \"%s.%s\"\n",filename.c_str(),suffix.c_str());
      
    // store status flag
    _file_export = true;
  }
  
  /**
   * @brief Use this function to complete the pdf export after plotting.
   */
  void completePdfExport()
  {
    assert(pipe);
    fprintf(pipe, "set output \n");
    _pdf_flag = false;
  }
  
  /**
   * @brief Change output to a common gnuplot window.
   */
  void setOutputToWindow()
  {
    #ifdef APPLE
      fprintf(pipe, "set term aqua enhanced\n"); // you can switch to "set term x11", but for me x11 does only work with xterm.
      fprintf(pipe, "TERM = \"set term aqua enhanced\"\n"); // define macro for further use
    #else
      fprintf(pipe, "set term wx noraise enhanced\n");   // set the terminal (output to window) ; output to svg etc works as well
							     // noraise: do not set window active if updated or refreshed
      fprintf(pipe, "TERM = \"set term wx noraise enhanced\"\n"); // define macro for further use
    #endif // set enhanced option to allow special letters (integral, greek, ...)      
	    
  }
  
  /**
   * @brief Custom the type of color and line style.
   */
  void setOptionsPlot(std::string color, int line_style)
  {          
     _options.color.append(color);
     _options.line_style = line_style;
     _options.show_grid = true;
     
     _change_options_plot = true;
  }
  
  void updateParameters(double* length, double* width)
{
   _L  = *length;
   _Wr = *width; 
}

void setDistanceRobotPlot(double dis)
{
  _THRESHOLD = dis; 
}

Eigen::VectorXd getRobotPosOnTrajectory(const Eigen::Ref<Eigen::MatrixXd> & states) 
{
  double x1 = static_cast<double>(states.col(0)[0]);  //! Take the initial point (X value)
  double y1 = static_cast<double>(states.col(0)[1]);  //! Take the initial point (Y value)
  double distance;
  Eigen::VectorXd posvec;
  int idx = 0;
  
  //! The first point is store
  posvec.conservativeResize(idx+1);
  posvec[0] = 1;
  idx++;   
  
  for(unsigned int i=0; i<states.cols(); ++i)
  {
      distance = std::sqrt(std::pow(x1 - states.col(i)[0],2) + std::pow(y1 - states.col(i)[1],2));
      //std::cout << "Distance between actual -> [" << i << "] = [" << distance << "]" << std::endl; 
      
      if(distance >= _THRESHOLD)
      {
	posvec.conservativeResize(idx+1);
	posvec[idx] = i;
	idx++;
	
	x1 = static_cast<double>(states.col(i)[0]); 
	y1 = static_cast<double>(states.col(i)[1]); 
	
	//std::cout << "Data stored..." << std::endl;
      }
  }
  
  //! The last position is also stored...
  posvec.conservativeResize(idx+1);
  posvec[idx] = states.cols()-1;    
  
  return posvec;
}


  
  
  /**
   * @brief Query status if export to file is enabled.
   * @return \c true, if export to file is enabled
   */
   bool isExportToFileEnabled() {return _file_export;}
    
protected:
  
  //! Plot a legend into a multiplot environment.
  void plotCustomKey(const std::vector<std::string>& keys);
    
  bool _pdf_flag = false; //!< \c true if PDF export is enabled.
  bool _file_export = false; //!< \c true if export to file is desired
  
  FILE *pipe = nullptr; //! Pipe to gnuplot
  
  bool _change_options_plot = false;
  
  PlotOptions _options;
  double _L;
  double _Wr;    
  double _radius = 0.1;
  
  Eigen::Vector4d _plot_range = Eigen::Vector4d(-1000, 1000, -100, -100);
  
  double _THRESHOLD = 2.0;
  
  double _xmin = -8;
  double _xmax = 8;
  double _ymin = -8;
  double _ymax = 8;
  
  //Footprint _robot;
     
};




} // end namespace teb
 
#include <teb_package/visualization/teb_plotter.hpp>

    
#endif // end ifdef RTW


#endif /* defined(__teb_package__teb_plotter__) */
