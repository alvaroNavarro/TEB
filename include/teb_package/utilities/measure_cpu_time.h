#ifndef __teb_package__measure_cpu_time__
#define __teb_package__measure_cpu_time__

#include <teb_package/base/typedefs.h>


//#if defined(WIN32) // -> seems to work with windows, ubuntu and mac
//  #define START_TIMER std::cout << "Timer support currently not supported for the selected operating system" << std::endl;
//  #define STOP_TIMER(name)  
//#else
  #include <chrono>
  #define START_TIMER auto start = std::chrono::high_resolution_clock::now();
  #ifdef NDEBUG // cast to microseconds is possible, too.
    #define STOP_TIMER(name)  PRINT_INFO("RUNTIME of " << name << ": " << \
	std::chrono::duration_cast<std::chrono::milliseconds>( \
		std::chrono::high_resolution_clock::now()-start \
	).count() << " ms "); 
  #else
    #define STOP_TIMER(name)  PRINT_INFO("RUNTIME of " << name << ": " << \
	std::chrono::duration_cast<std::chrono::milliseconds>( \
		std::chrono::high_resolution_clock::now()-start \
	).count() << " ms " << "(Compiled in Debug mode)"); 
  #endif
//#endif
    
/*      
namespace teb
{


} // end namespace teb
 */   


#endif /* defined(__teb_package__measure_cpu_time__) */
