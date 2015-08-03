#include <teb_package/visualization/teb_plotter.h>
#include <teb_package/simulation/simulator.h>

#include <string.h>

#ifndef RTW


namespace teb
{

/**
 * Constructs a new TebPlotter object.
 * A new pipe to gnuplot is initialized.
 * The default output is set to a new window.
 */  
TebPlotter::TebPlotter()
{
      std::string gnu_plot_path(GNUPLOT_PATH);
      gnu_plot_path = "\"" + gnu_plot_path + "\" -persist"; // add quotes (since path may containes spaces)
							    // add persist option to keep window open even though program quits.
      #ifdef WIN32
	  pipe = _popen(gnu_plot_path.c_str(), "w");
      #else
	  pipe = popen(gnu_plot_path.c_str(), "w"); // GNUPLOT_PATH is defined in CMakeLists.txt if gnuplot is found.
      #endif

      if (pipe == NULL)
      {
          PRINT_INFO("TebPlotter - Could not open pipe.");
          return;
      }
      fprintf(pipe, "set macros\n"); // enable macros

      setOutputToWindow(); // enable output to window by default
}

/**
 * Destructs the TebPlotter object.
 * The pipe to gnuplot is closed.
 */  
TebPlotter::~TebPlotter()
{
      // set output to nothing! (this is necessary if output is set to PDF in order to complete previously generated files)
      // PRINT_DEBUG_COND(_pdf_flag, "In order to complete the PDF correctly, run completePdfExport() after plotting.");
      completePdfExport();
      #ifdef WIN32
	  _pclose(pipe);
      #else
	  pclose(pipe);
      #endif
}


/**
 * Plot a single vector. The abscissa is chosen uniformly to 0 .. n
 * @param data Y data wrapped into an Eigen::Vector [n x 1]
 */ 
void TebPlotter::plot1DVector(const Eigen::Ref<const Eigen::VectorXd>& data)
{
      if (pipe == NULL)
      {
          PRINT_INFO("plot1DVector() - Could not open pipe.");
          return;
      }

      fprintf(pipe, "set key off\n"); //! disable legend
      fprintf(pipe, "set grid\n"); //! show grid
      fprintf(pipe, "plot '-' with lines\n"); //! plot type
      for(int i = 0; i < data.rows(); i++)       //! loop over the data
          fprintf(pipe, "%f\n", data[i]);           //! data terminated with \n
      fprintf(pipe, "%s\n", "e");             //! termination character
      fflush(pipe);                           //! flush the pipe
};
    

/**
 * Create a simple 2D plot using two different n-dim vectors for x and y
 * @param x x-data vector (abscissa): Eigen::Vector [n x 1]
 * @param y y-data vector (ordinate): Eigen::Vector [n x 1]
 * @param title String defining the title of the plot (optional)
 * @param xlabel String defining the label of the x-axis (optional)
 * @param ylabel String defining the label of the y-axis (optional)
 */ 
void TebPlotter::plot(const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& y, std::string title, std::string xlabel, std::string ylabel)
{
      assert(x.rows() == y.rows());
      if (pipe == NULL)
      {
		  PRINT_INFO("plot() - Could not open pipe.");
          return;
      }

      fprintf(pipe, "@TERM size 700,250\n"); // set size, TODO: move to class properties
      
      fprintf(pipe, "set key off\n"); //! disable legend
      fprintf(pipe, "set grid\n"); //! show grid
      
      if (!title.empty())  fprintf(pipe, "set title \"%s\"\n", title.c_str()); //! plot title
      if (!xlabel.empty())  fprintf(pipe, "set xlabel \"%s\"\n", xlabel.c_str()); //! plot xlabel
      if (!ylabel.empty())  fprintf(pipe, "set ylabel \"%s\"\n", ylabel.c_str()); //! plot ylabel
      
      fprintf(pipe, "plot '-' with lines\n"); //! plot type
      for(int i = 0; i < x.rows(); i++)       //! loop over the data
          fprintf(pipe, "%f %f\n", x[i], y[i]);   //! data terminated with \n
      fprintf(pipe, "%s\n", "e");             //! termination character
      fflush(pipe);                           //! flush the pipe
};

void TebPlotter::plotMultiLines(const PointPolygon& points)
{
   if(points.size() == 0)
   {
       PRINT_INFO("No data...");
       return;
   }      
   
   for(unsigned int i=0; i<points.size(); i=i+2)
   {
     std::cout << points.at(i)[0] << " " << points.at(i)[1]  << std::endl;
     std::cout << points.at(i+1)[0] << " " << points.at(i+1)[1] << std::endl;  
     
     plotSingleLine(points.at(i),points.at(i+1));
   }
   
}

void TebPlotter::plotSingleLine(const Eigen::Ref<const Eigen::Vector2d>& point1, const Eigen::Ref<const Eigen::Vector2d>& point2)
{
  if (pipe == NULL)
    {
	  PRINT_INFO("plot() - Could not open pipe.");
          return;
    }
        
    fprintf(pipe, "set linetype 1 linecolor rgb \"black\" linetype 1 linewidth 2\n");    
    fprintf(pipe, "plot '-' with lines \n");
    fprintf(pipe, "%f %f\n", static_cast<double>(point1[0]), static_cast<double>(point1[1]));
    fprintf(pipe, "%f %f\n", static_cast<double>(point2[0]), static_cast<double>(point2[1]));     
   
    fprintf(pipe,"e");
}
   


/**
 * Create a two column multiplot with a single line for each plot.
 * The number of rows for each column may differ.
 * 
 * @remarks No legend options implemented
 *
 * @test What happens if the right column contains more rows than the left one? (Alignment)
 * 
 * @param col1_time time axis for the left column [n1 x 1]
 * @param col1_data data for the left column: Each row corresponds to a single plot [rows1 x n1]
 * @param col2_time time axis for the right column [n2 x 1]
 * @param col2_data data for the right column: Each row corresponds to a single plot [rows2 x n2]
 * @param options Pointer to customized plot options given by a PlotOptions object.
 */ 
void TebPlotter::plotTwoCol(const Eigen::Ref<const Eigen::VectorXd>& col1_time, const Eigen::Ref<const Eigen::MatrixXd>& col1_data,
                            const Eigen::Ref<const Eigen::VectorXd>& col2_time, const Eigen::Ref<const Eigen::MatrixXd>& col2_data, PlotOptions* options)
{
    if (pipe == NULL)
    {
		PRINT_INFO("plotTwoCol() - Could not open pipe.");
        return;
    }
    const unsigned int col1_rows = (unsigned int) col1_data.rows();
	const unsigned int col2_rows = (unsigned int) col2_data.rows();
    const unsigned int n1 = (unsigned int) col1_data.cols();
    unsigned int n2 = (unsigned int) col2_data.cols();
    
    if (n1 != col1_time.size() || n2 != col2_time.size())
    {
		PRINT_INFO("plotTwoCol() - Cannot plot. Size mismatch between time vectors and data matrices.");
    }
    
    // remove last point if wanted (this removes the invalid control at time n for example)
    if (options && options->skip_last_value_right_column && n2>0)
    {
        --n2;
    }
    
    int max_rows = std::max(col1_rows,col2_rows);
    int max_cols = col2_time.rows()==0 ? 1 : 2;
    
   
    const double t_margin = 20.0; // actual t_margin = t_margin + tb_space
    const double b_margin = 60.0;
    const double tb_space = 20;
    const double height_single = 150;
    const double height = height_single*(double(max_rows)) + t_margin + b_margin + tb_space*double(max_rows);
    const double l_margin = 0.0; // actual l_margin = l_margin + lr_space
    const double r_margin = 20.0;
    const double lr_space = 90;
    const double width_single = 600;
    const double width = double(max_cols)*width_single + l_margin + r_margin + lr_space*double(max_cols);
    
    
    // functions to help set top/bottom margins
    auto top   = [&height,&t_margin,&b_margin,&tb_space] (int i,int rows)  {return 1.0 - (t_margin+tb_space+(height-t_margin-b_margin)*(i-1)/rows)/height;};
    auto bot   = [&height,&t_margin,&b_margin,&tb_space] (int i,int rows)  {return 1.0 - (t_margin+(height-t_margin-b_margin)*i/rows)/height;};
    auto left  = [&width,&l_margin,&r_margin,&lr_space] (int j,int cols)  {return (l_margin+lr_space+(width-r_margin-l_margin)*(j-1)/cols)/width;};
    auto right = [&width,&l_margin,&r_margin,&lr_space] (int j,int cols)  {return (l_margin+(width-r_margin-l_margin)*j/cols)/width;};
    

    fprintf(pipe, "set lmargin at screen 0.11\n");
    fprintf(pipe, "set rmargin at screen 0.4\n");
    
    // setup terminal
    fprintf(pipe, "@TERM size %f, %f\n",width,height);
    
    //fprintf(pipe, "set multiplot layout %d, %d columnsfirst\n",max_rows,max_cols);
    if (!options || options->title.empty())
        fprintf(pipe, "set multiplot \n"); // start multiplot environment
    else
        fprintf(pipe, "set multiplot title \"%s\"\n",options->title.c_str()); // start multiplot environment with single title
    
    // plot options
    fprintf(pipe, "set grid\n");
    fprintf(pipe, "unset key\n");
   
    
    const std::string disable_xtics("set xtics format\'\'\n"); // we could use "unset xtics" here, but that would remove column grid lines as well.
    const std::string enable_xtics("set xtics format  \'%.1f\'\n set xlabel \"t [s]\n");


        
    // For the first assume that it is always p>q (plot all states first (columnsfist) and then inputs.
    // Since skipping plots (skip multiplot next) is only supportet with gnuplot 4.7+
    for (unsigned int data_idx = 0; data_idx < col1_rows; ++data_idx)
    {
        // Plot column 1
        fprintf(pipe, "set tmargin at screen %f\n",top(data_idx+1,max_rows));
        fprintf(pipe, "set bmargin at screen %f\n",bot(data_idx+1,max_rows));
        fprintf(pipe, "set lmargin at screen %f\n",left(1,max_cols));
        fprintf(pipe, "set rmargin at screen %f\n",right(1,max_cols));
        
	if (options && data_idx<options->ylabels.size())
	{
	  fprintf(pipe, "set ylabel \"%s\"\n",options->ylabels.at(data_idx).c_str());
	}
	else // use default label
	{
	  fprintf(pipe, "set ylabel \"x_%d (t)\"\n",data_idx+1);
	}
	  
        if (data_idx<col1_rows-1) fprintf(pipe, "%s",disable_xtics.c_str());
        else fprintf(pipe, "%s", enable_xtics.c_str());
        
        fprintf(pipe, "plot '-' with lines\n"); // plot type
        for(unsigned int i = 0; i < n1; ++i)             // loop over the data
        {
            fprintf(pipe, "%f %f\n", col1_time(i), col1_data(data_idx,i)); // data terminated with \n
        }
        fprintf(pipe, "%s\n", "e");             // termination character
    }
    
    for (unsigned int data_idx = 0; data_idx < col2_rows; ++data_idx)
    {
        // Plot column 2
        fprintf(pipe, "set tmargin at screen %f\n",top(data_idx+1,max_rows));
        fprintf(pipe, "set bmargin at screen %f\n",bot(data_idx+1,max_rows));
        fprintf(pipe, "set lmargin at screen %f\n",left(2,max_cols));
        fprintf(pipe, "set rmargin at screen %f\n",right(2,max_cols));
        
	if (options && col1_rows+data_idx<options->ylabels.size())
	{
	  fprintf(pipe, "set ylabel \"%s\"\n",options->ylabels.at(col1_rows+data_idx).c_str());
	}
	else // use default label
	{
	  fprintf(pipe, "set ylabel \"u_%d (t)\"\n",data_idx+1);
	}
        
        
        if (data_idx<col2_rows-1) fprintf(pipe, "%s",disable_xtics.c_str());
        else fprintf(pipe, "%s", enable_xtics.c_str());
        
        fprintf(pipe, "plot '-' with lines\n"); // plot type
        
        for(unsigned int i = 0; i < n2; ++i)             // loop over the data
        {
            fprintf(pipe, "%f %f\n", col2_time(i), col2_data(data_idx,i)); // data terminated with \n
        }
        fprintf(pipe, "%s\n", "e");             // termination character
    }
    
    fprintf(pipe, "unset multiplot\n");
    
    fflush(pipe);                           // flush the pipe

}
    
    
/**
 * Create a two column multiplot with multiple lines for each plot.
 * The number of rows for each column may differ.
 * 
 * One element of the col1 arguments stores the complete information for all subplots.
 * Other elements extend each plot by new lines.
 * E.g:
 * - \c col1_data.at(0) contains 3 rows, that results 3 subplots / rows for the left column.
 *   Each row specifies one line for each plot.
 * - \c col1_data.at(1) contains 3 rows as well. Each line is added to its corresponding subplots. \n
 *    This requires the same number of samples (columns) between \c col1_data.at(i) and col1_time.at(i).
 *
 * @param col1_time container of time data for the lines in the left column
 * @param col1_data container of y-data for the lines in the left colum
 * @param col2_time container of time data for the lines in the right column
 * @param col2_data container of y-data for the lines in the right colum
 * @param options Pointer to customized plot options given by a PlotOptions object.
 */     
void TebPlotter::plotTwoCol(const std::vector<const Eigen::VectorXd*>& col1_time, const std::vector<const Eigen::MatrixXd*>& col1_data,
                            const std::vector<const Eigen::VectorXd*>& col2_time, const std::vector<const Eigen::MatrixXd*>& col2_data, PlotOptions* options)
{
    if (pipe == NULL)
    {
		PRINT_INFO("plotTwoCol() - Could not open pipe.");
        return;
    }
    
    if (col1_time.size()!=col1_data.size() || col2_time.size()!=col2_data.size())
    {
		PRINT_INFO("plotTwoCol() - Data mismatch between time and data vectors.");
        return;
        
    }
    
    // check dimensions && determine y-range [y_min, y_max]
    const unsigned int col1_rows = (unsigned int) col1_data.front()->rows();
    const unsigned int col2_rows = (unsigned int) col2_data.front()->rows();
    
    // get max time t
    double col1_t_max = 0;
    double col2_t_max = 0;
    for (const Eigen::VectorXd* t1 : col1_time)
    {
      double t_max = (*t1)[t1->size()-1];
      if ( t_max > col1_t_max) col1_t_max = t_max;
    }
    for (const Eigen::VectorXd* t2 : col2_time)
    {
      double t_max = (*t2)[t2->size()-1];
      if ( t_max > col2_t_max) col2_t_max = t_max;
    }
    
    
    // get y range
    std::vector< std::pair<double,double> > col1_y(col1_rows, std::make_pair(0.,0.));
    std::vector< std::pair<double,double> > col2_y(col2_rows, std::make_pair(0.,0.));
    
    for (unsigned int i=0; i<col1_data.size(); ++i)
    {
        if (col1_data.at(i)->cols() != col1_time.at(i)->size() || col1_data.at(i)->rows() != col1_rows)
        {
			PRINT_INFO("plotTwoCol() - Cannot plot. Size mismatch between time vectors and data matrices or data matrices inside vectors (col1_data).");
            return;
        }
        // Get y range
        for (unsigned int j=0; j<col1_data.at(i)->rows(); ++j)
        {
            double min_coeff = col1_data.at(i)->row(j).minCoeff();
            double max_coeff = col1_data.at(i)->row(j).maxCoeff();
            if ( min_coeff < col1_y.at(j).first) col1_y.at(j).first = min_coeff;
            if ( max_coeff > col1_y.at(j).second) col1_y.at(j).second = max_coeff;
        }
    }
    for (unsigned int i=0; i<col2_data.size(); ++i)
    {
        if (col2_data.at(i)->cols() != col2_time.at(i)->size() || col2_data.at(i)->rows() != col2_rows)
        {
			PRINT_INFO("plotTwoCol() - Cannot plot. Size mismatch between time vectors and data matrices or data matrices inside vectors (col2_data).");
            return;
        }
        // Get y range
        for (unsigned int j=0; j<col2_data.at(i)->rows(); ++j)
        {
            double min_coeff = col2_data.at(i)->row(j).minCoeff();
            double max_coeff = col2_data.at(i)->row(j).maxCoeff();
            if ( min_coeff < col2_y.at(j).first) col2_y.at(j).first = min_coeff;
            if ( max_coeff > col2_y.at(j).second) col2_y.at(j).second = max_coeff;
        }
    }
    

    int max_rows = std::max(col1_rows,col2_rows);
    int max_cols = col2_time.empty() ? 1 : 2;
    
    // check legend
    double legend_height = 0;
    if (options)
    {
      if (options->legend)
      {
        legend_height = 50;
      }
    }
    
    const double t_margin = 20.0; // actual t_margin = t_margin + tb_space
    const double b_margin = 60.0;
    const double tb_space = 20;
    const double height_single = 150;
    const double height = height_single*(double(max_rows)) + t_margin + b_margin + tb_space*double(max_rows) + legend_height;
    const double l_margin = 0.0; // actual l_margin = l_margin + lr_space
    const double r_margin = 20.0;
    const double lr_space = 90;
    const double width_single = 600;
    const double width = double(max_cols)*width_single + l_margin + r_margin + lr_space*double(max_cols);
    
    const double range_scale = 1.1; // increase ranges by 10%
    
    // functions to help set top/bottom margins
    auto top   = [&height,&t_margin,&b_margin,&tb_space,&legend_height] (int i,int rows)  {return 1.0 - (t_margin+tb_space+(height-t_margin-b_margin-legend_height)*(i-1)/rows)/height;};
    auto bot   = [&height,&t_margin,&b_margin,&tb_space,&legend_height] (int i,int rows)  {return 1.0 - (t_margin+(height-t_margin-b_margin-legend_height)*i/rows)/height;};
    auto left  = [&width,&l_margin,&r_margin,&lr_space] (int j,int cols)  {return (l_margin+lr_space+(width-r_margin-l_margin)*(j-1)/cols)/width;};
    auto right = [&width,&l_margin,&r_margin,&lr_space] (int j,int cols)  {return (l_margin+(width-r_margin-l_margin)*j/cols)/width;};
    
    
    fprintf(pipe, "set lmargin at screen 0.11\n");
    fprintf(pipe, "set rmargin at screen 0.4\n");
    
    // setup terminal
    fprintf(pipe, "@TERM size %f, %f\n",width,height);
    
    //fprintf(pipe, "set multiplot layout %d, %d columnsfirst\n",max_rows,max_cols);
    if (!options || options->title.empty())
        fprintf(pipe, "set multiplot \n"); // start multiplot environment
    else
        fprintf(pipe, "set multiplot title \"%s\"\n",options->title.c_str()); // start multiplot environment with single title
    
    // plot options
    fprintf(pipe, "set grid\n");
    fprintf(pipe, "unset key\n");

    
    
    
    const std::string disable_xtics("set xtics format\'\'\n unset xlabel\n"); // we could use "unset xtics" here, but that would remove column grid lines as well.
    const std::string enable_xtics("set xtics format  \'%.1f\'\n set xlabel \"t [s]\n");
    
    
    // Plot column 1
    // For the first assume that it is always p>q (plot all states first (columnsfist) and then inputs.
    // Since skipping plots (skip multiplot next) is only supportet with gnuplot 4.7+
    for (unsigned int data_idx = 0; data_idx < col1_rows; ++data_idx)
    {
        fprintf(pipe, "set tmargin at screen %f\n",top(data_idx+1,max_rows));
        fprintf(pipe, "set bmargin at screen %f\n",bot(data_idx+1,max_rows));
        fprintf(pipe, "set lmargin at screen %f\n",left(1,max_cols));
        fprintf(pipe, "set rmargin at screen %f\n",right(1,max_cols));
        
	if (options && data_idx<options->ylabels.size())
	{
	  fprintf(pipe, "set ylabel \"%s\"\n",options->ylabels.at(data_idx).c_str());
	}
	else // use default label
	{
	  fprintf(pipe, "set ylabel \"x_%d (t)\"\n",data_idx+1);
	}
        
	// set xrange /trage
	fprintf(pipe, "set xr [0:%f]\n", range_scale*col1_t_max);
	
        // set yrange
	if (col1_y.at(data_idx).first == col1_y.at(data_idx).second) // if no range is given (y_min=y_max), set range to (y_min-0.5 : y_max+0.5);
	{
	  fprintf(pipe, "set yr [%f:%f]\n",range_scale*col1_y.at(data_idx).first-0.5, range_scale*col1_y.at(data_idx).second+0.5);
	}
	else
	{
	  fprintf(pipe, "set yr [%f:%f]\n",range_scale*col1_y.at(data_idx).first, range_scale*col1_y.at(data_idx).second);
	}
        
        if (data_idx<col1_rows-1) fprintf(pipe, "%s",disable_xtics.c_str());
        else fprintf(pipe, "%s", enable_xtics.c_str());
        
        for (unsigned int line_idx = 0; line_idx < col1_data.size(); ++line_idx)
        {
            // Plot column 1 for each line (lines: vector)
	              
            fprintf(pipe, "plot '-' with lines lt %d \n",line_idx+1); // plot type with line type depending on line_idx
            for(unsigned int i = 0; i < col1_data.at(line_idx)->cols(); ++i)             // loop over the data
            {
                fprintf(pipe, "%f %f\n", col1_time.at(line_idx)->coeffRef(i), col1_data.at(line_idx)->coeffRef(data_idx,i)); // data terminated with \n
            }
            fprintf(pipe, "e\n");             // termination character
        }
    }
    
    for (unsigned int data_idx = 0; data_idx < col2_rows; ++data_idx)
    {
        // Plot column 2
        fprintf(pipe, "set tmargin at screen %f\n",top(data_idx+1,max_rows));
        fprintf(pipe, "set bmargin at screen %f\n",bot(data_idx+1,max_rows));
        fprintf(pipe, "set lmargin at screen %f\n",left(2,max_cols));
        fprintf(pipe, "set rmargin at screen %f\n",right(2,max_cols));
        
	if (options && col1_rows+data_idx<options->ylabels.size())
	{
	  fprintf(pipe, "set ylabel \"%s\"\n",options->ylabels.at(col1_rows+data_idx).c_str());
	}
	else // use default label
	{
	  fprintf(pipe, "set ylabel \"u_%d (t)\"\n",data_idx+1);
	}
        
        
        // set xrange /trage
        fprintf(pipe, "set xr [0:%f]\n", range_scale*col2_t_max);
	
        // set yrange
        if (col1_y.at(data_idx).first == col1_y.at(data_idx).second) // if no range is given (y_min=y_max), set range to (y_min-0.5 : y_max+0.5);
        {
            fprintf(pipe, "set yr [%f:%f]\n",range_scale*col2_y.at(data_idx).first-0.5, range_scale*col2_y.at(data_idx).second+0.5);
        }
        else
        {
            fprintf(pipe, "set yr [%f:%f]\n",range_scale*col2_y.at(data_idx).first, range_scale*col2_y.at(data_idx).second);
        }
        
        if (data_idx<col2_rows-1) fprintf(pipe, "%s",disable_xtics.c_str());
        else fprintf(pipe, "%s", enable_xtics.c_str());
        
        
        for (unsigned int line_idx = 0; line_idx < col2_data.size(); ++line_idx)
        {
            fprintf(pipe, "plot '-' with lines lt %d \n",line_idx+1); // plot type with line type depending on line_idx
            
            unsigned int no_points = (unsigned int) col2_data.at(line_idx)->cols();
            // remove last point if wanted (this removes the invalid control at time n for example)
            if (options && options->skip_last_value_right_column && no_points>0)
            {
                --no_points;
            }
            for(unsigned int i = 0; i < no_points; ++i)             // loop over the data
            {
                fprintf(pipe, "%f %f\n", col2_time.at(line_idx)->coeffRef(i), col2_data.at(line_idx)->coeffRef(data_idx,i)); // data terminated with \n
            }
            fprintf(pipe, "e\n");             // termination character
        }
    }
    
    // prepare margins for the following legend plot
    if (options && options->legend)
    {
        fprintf(pipe, "set tmargin at screen %f\n", bot(max_rows,max_rows) - 4*tb_space/height);
        fprintf(pipe, "set bmargin at screen 0\n");
        fprintf(pipe, "set lmargin at screen %f\n", lr_space/width);
        fprintf(pipe, "set rmargin at screen %f\n", 1-r_margin/width);
        //fprintf(pipe, "set lmargin at screen %f\n",left(1,max_cols));
        //fprintf(pipe, "set rmargin at screen %f\n",right(1,max_cols));
        plotCustomKey(options->legend_entries);
    }
    fprintf(pipe, "unset multiplot\n");
    fflush(pipe);                           // flush the pipe
}
    
    
    
/**
 * This function visualizes the non-zeros of a given sparse matrix with a marker
 * at the specific non-zero position.
 * 
 * @param  matrix Matrix given as an Eigen::Matrix data type.
 */
void TebPlotter::spyMatrix(const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{
    if (pipe == NULL)
    {
		PRINT_INFO("spyMatrix() - Could not open pipe.");
        return;
    }
    
    int nnz = 0; // count nonzeros
    const unsigned int rows = (unsigned int)  matrix.rows();
    const unsigned int cols = (unsigned int) matrix.cols();
    
    fprintf(pipe, "set key off\n"); // disable legend
    fprintf(pipe, "set grid\n"); // show grid
    fprintf(pipe, "set xrange [0:%d]\n", int(cols-1)); // xrange (start with index 0)
    fprintf(pipe, "set yrange [0:%d]\n", int(rows-1)); // yrange (start with index 0)
    fprintf(pipe, "plot '-' with labels tc rgb \"red\"\n"); // plot type
    for(unsigned int i = 0; i < rows; ++i)       // loop over the data
    {
        for (unsigned int j=0;j < cols; ++j)
        {
            if (matrix.coeffRef(i,j)!=0)
            {
                nnz++;
                fprintf(pipe, "%d %d x\n",j, rows-i-1); // plot only nonzeros, Mark with "x", set (0,0) to top left corner
            }
        }
    }
    fprintf(pipe, "%s\n", "e");             // terminate
    
    // show number of nonzeros at x-axis:
    fprintf(pipe, "set xlabel \"nnz: %d (%.1f%%)\"\n",nnz,double(nnz)/double(matrix.size())*100);
    fprintf(pipe, "refresh\n"); // refresh figure to show text
    
    fflush(pipe);
}
    

/**
 * Use this function to plot a custom legend into a multicol environment. \
 * Specify location before calling this function by setting gnuplot margins
 * \c tmargin, \c bmargin, \c lmargin and \c rmargin.
 * 
 * @param keys Vector of legend strings (for each line)
 */
void TebPlotter::plotCustomKey(const std::vector<std::string>& keys)
{
    fprintf(pipe, "set xrange [0:1]\n");
    fprintf(pipe, "set yrange [0:1]\n");
    fprintf(pipe, "unset tics\n");
    fprintf(pipe, "unset xlabel\n");
    fprintf(pipe, "unset ylabel\n");
    fprintf(pipe, "set border 0\n");
    
    const double char_width = 0.0042;
    const double dx = 0.07;
    const double dy = 0.6;
    double x = 0;
    double y = 1;

    for (unsigned int i=0; i<keys.size(); ++i)
    {
        fprintf(pipe, "plot '-' with labels left offset 1,-0.2 point pt 7 lt %d \n",i+1); //! WORKS ONLY IN MULTI PLOT ENVIRONMENT!

        fprintf(pipe, "%f %f \"%s\"\n", x, y, keys.at(i).c_str());
        x += double(keys.at(i).size()) * char_width + dx;
        if (x > 1.)
        {
            x = 0;
            y -= dy;
        }
        
        fprintf(pipe, "%s\n", "e");             //! terminate
    }

    
}


void TebPlotter::plotFigureObstacle(const PointCircle& point_obs)
{
  int idx = 1;  
  
  if (pipe == NULL)
   {
 	std::cout << "plotFigureObstacle() - Could not open pipe." << std::endl;
	return;
   }  
   
   for (auto &vec : point_obs)
  {
     fprintf(pipe, "set label %i \" %i\" at %f,%f tc lt 3 \n", idx, idx, vec[0], vec[1] + vec[2]/1.8);
     idx++; 
  }  
      
  //fprintf(pipe, "set multiplot \n");    
  fprintf(pipe, "set style fill solid 1 \n");              
  fprintf(pipe, "plot '-' with circles lc rgb \"blue\" \n");
  
  for (auto &vec : point_obs)       //! loop over the data  
 	fprintf(pipe, "%f %f 0.02 \n", vec[0], vec[1]);                   //! add coordinates of polygon points		      
  
	
  fprintf(pipe, "%s\n", "e");             //! termination character	
  
  fprintf(pipe, "set style fill transparent solid 0.1 noborder \n");	
  fprintf(pipe, "plot '-' with circles lc rgb \"blue\" \n");
  
  for (auto &vec : point_obs)       //! loop over the data
 	fprintf(pipe, "%f %f %f \n", vec[0], vec[1], vec[2]/1.8);                   //! add coordinates of polygon points	
    
  fprintf(pipe, "%s\n", "e");             //! termination character
  //fprintf(pipe, "unset multiplot \n");
}


/**
 * Use this function to plot the robot over the trajectory given a coordinate point and its orientation. \
 * 
 * @param x 	coordinate in X axis
 * @param y 	coordinate in Y axis
 * @param theta Robot orientation in the specific point
 */


void TebPlotter::plotPolygon(const PointPolygon& data, bool front = true, std::string string = "")
{
   if (pipe == NULL)
   {
 	std::cout << "plotPolygon() - Could not open pipe." << std::endl;
	return;
   }            
	
   fprintf(pipe, "set object 1 polygon from "); //! define object
   
   for (auto &vec : data)       //! loop over the data
 	fprintf(pipe, "%f,%f to ", vec[0], vec[1]);                   //! add coordinates of polygon points

   fprintf(pipe, "%f,%f\n", data.front()[0], data.front()[1]);        //! add the first point again to close the polygon
   
   if(string == "transparent")
       fprintf(pipe, "set object 1 fc rgb \"%s\" fillstyle empty border lt %i\n", _options.color.c_str(), _options.line_style);//! Set options
   else
       fprintf(pipe, "set object 1 fc rgb \"%s\" fillstyle solid 1.0 border lt %i\n", _options.color.c_str(), _options.line_style);//! Set options


   //! Plot a line to make the polygon show up. This might be someway easier to handle...

   fprintf(pipe, "set style line 5 linecolor rgb \"black\" linetype 1 linewidth 0.1\n");
   
   if(front)
   {
      fprintf(pipe, "set arrow from ");
      fprintf(pipe, "%f,%f to ", data.front()[0], data.front()[1]);
      fprintf(pipe, "%f,%f ",   data.at(1)[0], data.at(1)[1]);
      fprintf(pipe, "nohead linecolor rgb \"black\" linetype 1 linewidth 2\n");  
   }
   
   fprintf(pipe, "plot '-' ls 5 with lines\n"); //! plot type

   fprintf(pipe, "%f %f\n", data.front()[0], data.front()[1]); 
   fprintf(pipe, "%f %f\n", data.back()[0], data.back()[1]);
   fprintf(pipe, "%s\n", "e");             //! termination character
         

  }
  
  void TebPlotter::plotCircle(const Eigen::Ref<const Eigen::Vector3d>& point, std::string str)
  {
      if (pipe == NULL)
   {
 	std::cout << "plotCircle() - Could not open pipe." << std::endl;
	return;
   }  
   
   fprintf(pipe, "set style fill solid 1 \n");              
   fprintf(pipe, "plot '-' with circles lc rgb \"%s\" \n", str.c_str());
     
   fprintf(pipe, "%f %f %f \n", point[0], point[1], point[2]);                   //! add coordinates of polygon points		      
  	
   fprintf(pipe, "%s\n", "e");             //! termination character
   
  }
      
   void TebPlotter::setPlotRange()
   {
     Eigen::Vector4d vec_points(_xmin,_xmax,_ymin,_ymax);     	  
     setPlotRange(vec_points);
   }
   
   void TebPlotter::setPlotRange(const Eigen::Ref<Eigen::Vector4d>& vec)
   {
     if(pipe == NULL)
     {
        std::cout << "setPlotRange() - Could not open pipe. " << std::endl;
        return;
     }
     
     // Store    
    _plot_range = vec + Eigen::Vector4d(-5,5,-5,5);

     // Set range in gnuplot
     fprintf(pipe, "set xrange [%f:%f] \n", _plot_range[0], _plot_range[1]);
     fprintf(pipe, "set yrange [%f:%f] \n", _plot_range[2], _plot_range[3]);
     
   }
    
} // end namespace teb


#endif // end ifdef RTW