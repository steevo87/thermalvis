IF(USE_DOXYGEN AND doxygen_FOUND)
	MESSAGE(STATUS "Will generate docs at ${PROJECT_BINARY_DIR}.")
	configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile @ONLY IMMEDIATE)

	IF(FORCE_DOCS)
		add_custom_target(Docs ALL
			COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
			SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
		)
	ELSE()
		add_custom_target(Docs
			COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
			SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
		)
	ENDIF()
ENDIF()
