#include <teb_package/visualization/teb_plotter.h>

namespace teb
{

/**
 * Use this function to plot the state and control input sequence of the TebController
 * given as TebController::TEBVector type.
 * 
 * @remarks Based on http://stackoverflow.com/questions/14712251/place-key-below-multiplot-graph-in-gnuplot 
 * 	    and extended to two column multiplots
 *
 * @param state_seq State sequence (Vectors of type StateVertex)
 * @param ctrl_seq Ccontrol input sequence (Vectors of type ControlVertex)
 * @param dt Time interval between two consecutive states and control inputs \f$ \Delta T \f$.
 * @tparam TebVec Type of the TebVec (deduced by the compiler).
 */
template <typename StateSeq, typename ControlSeq>
void TebPlotter::plotTEB(const StateSeq& state_seq, ControlSeq& ctrl_seq, double dt)
{
    if (state_seq.empty() && ctrl_seq.empty())
    {
      return; 
    }
    if (pipe == NULL)
    {
		PRINT_INFO("plotTEB() - Could not open pipe.");
      return;
    }

    const int p = state_seq.empty() ? 0 : state_seq.front().DimStates;
    const int q = ctrl_seq.empty() ? 0 : ctrl_seq.front().DimControls;
    int max_rows = std::max(p,q);
    int max_cols = q==0 ? 1 : 2;
    unsigned int n = (unsigned int) state_seq.size();
    unsigned int m = (unsigned int) ctrl_seq.size();
    
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
       
    fprintf(pipe, "set lmargin at screen 0.11\n");
    fprintf(pipe, "set rmargin at screen 0.4\n");  
    
    
    // functions to help set top/bottom margins
    auto top   = [&height,&t_margin,&b_margin,&tb_space] (int i,int rows)  {return 1.0 - (t_margin+tb_space+(height-t_margin-b_margin)*(i-1)/rows)/height;};
    auto bot   = [&height,&t_margin,&b_margin,&tb_space] (int i,int rows)  {return 1.0 - (t_margin+(height-t_margin-b_margin)*i/rows)/height;};
    auto left  = [&width,&l_margin,&r_margin,&lr_space] (int j,int cols)  {return (l_margin+lr_space+(width-r_margin-l_margin)*(j-1)/cols)/width;};
    auto right = [&width,&l_margin,&r_margin,&lr_space] (int j,int cols)  {return (l_margin+(width-r_margin-l_margin)*j/cols)/width;};
    
    // setup terminal
    fprintf(pipe, "@TERM size %f, %f\n",width,height);
   
    
    fprintf(pipe, "set multiplot layout %d, %d columnsfirst title \"TEB States (dt: %.3f, n: %d)\"\n",max_rows,q==0?1:2,dt,n);
    
    // plot options
    fprintf(pipe, "set grid\n");
    fprintf(pipe, "unset key\n");
    
    const std::string disable_xtics("set xtics format\'\'\n"); // we could use "unset xtics" here, but that would remove column grid lines as well.
    const std::string enable_xtics("set xtics format  \'%.1f\'\n set xlabel \"t [s]\n");
    
    // For the first assume that it is always p>q (plot all states first (columnsfist) and then inputs. 
    // Since skipping plots (skip multiplot next) is only supportet with gnuplot 4.7+
    for (int state_idx = 0; state_idx < p; ++state_idx)
    {
      // Plot state
      fprintf(pipe, "set tmargin at screen %f\n",top(state_idx+1,max_rows));
      fprintf(pipe, "set bmargin at screen %f\n",bot(state_idx+1,max_rows));
      fprintf(pipe, "set lmargin at screen %f\n",left(1,max_cols));
      fprintf(pipe, "set rmargin at screen %f\n",right(1,max_cols));
      
      fprintf(pipe, "set ylabel \"x_%d (t)\"\n",state_idx+1);

      if (state_idx<p-1) fprintf(pipe, "%s",disable_xtics.c_str());
      else fprintf(pipe, "%s", enable_xtics.c_str());
      
      fprintf(pipe, "plot '-' with lines\n"); // plot type
      for(unsigned int i = 0; i < n; ++i)             // loop over the data
      {
	fprintf(pipe, "%f %f\n", double(i)*dt, state_seq.at(i).states()[state_idx]); // data terminated with \n
      }	
      fprintf(pipe, "%s\n", "e");             // termination character   
    }
    
    for (int ctrl_idx = 0; ctrl_idx < q; ++ctrl_idx)
    {
      // Plot controls
      fprintf(pipe, "set tmargin at screen %f\n",top(ctrl_idx+1,max_rows));
      fprintf(pipe, "set bmargin at screen %f\n",bot(ctrl_idx+1,max_rows));
      fprintf(pipe, "set lmargin at screen %f\n",left(2,max_cols));
      fprintf(pipe, "set rmargin at screen %f\n",right(2,max_cols));
      
      fprintf(pipe, "set ylabel \"u_%d (t)\"\n",ctrl_idx+1);
	
      if (ctrl_idx<q-1) fprintf(pipe, "%s",disable_xtics.c_str());
      else fprintf(pipe, "%s", enable_xtics.c_str());
      
      fprintf(pipe, "plot '-' with lines\n"); // plot type
      
      for(unsigned int i = 0; i < m; ++i)             // loop over the data
      {
	fprintf(pipe, "%f %f\n", double(i)*dt, ctrl_seq.at(i).controls()[ctrl_idx]); // data terminated with \n
      }
      fprintf(pipe, "%s\n", "e");             // termination character   
    }   
    
    
    
    fprintf(pipe, "unset multiplot\n");
    fflush(pipe);                           // flush the pipe

}

} // end namespace teb