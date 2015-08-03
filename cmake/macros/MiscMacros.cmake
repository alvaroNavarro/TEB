MACRO(CHANGE_MD_TO_MT flag_string)

	# get the name of the flag_string
    SET(flags_name  ${flag_string})
	
	# get the value of flag_string
    SET(flags_value ${${flag_string}})
	
	# Call cmakes replace command
    STRING(REPLACE /MD /MT tmp "${flags_value}")
	
    STRING(COMPARE NOTEQUAL "${tmp}" "${flags_value}" replaced)
	
    if("${replaced}")
        UNSET(${flags_name} CACHE)
        SET(${flags_name} ${tmp} CACHE STRING "Replaced /MD with /MT.")
    endif()
ENDMACRO()