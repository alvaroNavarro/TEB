#Author: Christoph Rösmann <christoph.roesmann@tu-dortmund.de>

MACRO(GET_DIR_FILE StringIn DirOut FilenameOut)

# parse element string to get directory and name (only one subdirectory supported)
  string(FIND ${ELEMENT} "/" EL_PATHPOS)
  if(${EL_PATHPOS} EQUAL -1) # set empty if no path found
    set(EL_PATH "")
    set(EL_FILE ${ELEMENT})
  else(${EL_PATHPOS} EQUAL -1)
   MATH(EXPR EL_PATHPOS "${EL_PATHPOS}+1") # include slash
   string(SUBSTRING ${ELEMENT} 0 ${EL_PATHPOS} EL_PATH) # copy path
   string(SUBSTRING ${ELEMENT} ${EL_PATHPOS} -1 EL_FILE) # copy path
  endif(${EL_PATHPOS} EQUAL -1)


MARK_AS_ADVANCED(
  DirOut
  FilenameOut
)

ENDMACRO(GET_DIR_FILE StringIn DirOut FilenameOut)


