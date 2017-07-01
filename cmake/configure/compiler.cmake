if(IS_WINDOWS)
    # Configure for Visual Studio
    IF( 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 10")              OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 10 Win64")        OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 10 2010")         OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 10 Win64 2010")   OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 10 2010 Win64") )
            SET(msvc_ABBREVIATION "msvc2010")
			IF ( IS_64_BIT )
				SET(msvc_LIB "lib64-msvc-10.0")
			ELSE()
				SET(msvc_LIB "lib32-msvc-10.0")
			ENDIF()
    ELSEIF( 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 11")              OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 11 Win64")        OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 11 2012")         OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 11 Win64 2012")   OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 11 2012 Win64") )
            SET(msvc_ABBREVIATION "msvc2012")
			IF ( IS_64_BIT )
				SET(msvc_LIB "lib64-msvc-11.0")
			ELSE()
				SET(msvc_LIB "lib32-msvc-11.0")
			ENDIF()
    ELSEIF( 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 12")              OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 12 Win64")        OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 12 2013")         OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 12 Win64 2013")   OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 12 2013 Win64") )
            SET(msvc_ABBREVIATION "msvc2013")
			IF ( IS_64_BIT )
				SET(msvc_LIB "lib64-msvc-12.0")
			ELSE()
				SET(msvc_LIB "lib32-msvc-12.0")
			ENDIF()
    ELSEIF( 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 14")              OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 14 Win64")        OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 14 2015")         OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 14 Win64 2015")   OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 14 2015 Win64") )
            SET(msvc_ABBREVIATION "msvc2015")
			IF ( IS_64_BIT )
				SET(msvc_LIB "lib64-msvc-14.0")
			ELSE()
				SET(msvc_LIB "lib32-msvc-14.0")
			ENDIF()
    ELSEIF( 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 15")              OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 15 Win64")        OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 15 2017")         OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 15 Win64 2015")   OR 
        ("${CMAKE_GENERATOR}" STREQUAL "Visual Studio 15 2017 Win64") )
            SET(msvc_ABBREVIATION "msvc2017")
			IF ( IS_64_BIT )
				SET(msvc_LIB "lib64-msvc-15.0")
			ELSE()
				SET(msvc_LIB "lib32-msvc-15.0")
			ENDIF()
    ENDIF()
    string(TOUPPER ${msvc_ABBREVIATION} MSVC_ABBREVIATION_CAPS)
endif()
