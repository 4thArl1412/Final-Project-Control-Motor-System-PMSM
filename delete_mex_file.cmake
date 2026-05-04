# Script to delete MEX file with retry logic
# Usage: cmake -P delete_mex_file.cmake <file_path>

if(ARGC LESS 1)
    message(FATAL_ERROR "No file path provided")
endif()

set(MEX_FILE "${ARGV0}")

if(EXISTS "${MEX_FILE}")
    message(STATUS "Attempting to delete MEX file: ${MEX_FILE}")

    # Try to delete with multiple attempts and waits
    set(MAX_ATTEMPTS 5)
    set(ATTEMPT 1)

    while(ATTEMPT LESS_EQUAL ${MAX_ATTEMPTS})
        message(STATUS "Delete attempt ${ATTEMPT}/${MAX_ATTEMPTS}...")

        # Attempt to remove the file
        file(REMOVE "${MEX_FILE}")

        # Check if file was deleted
        if(NOT EXISTS "${MEX_FILE}")
            message(STATUS "MEX file deleted successfully")
            break()
        endif()

        # If file still exists and not the last attempt, wait and retry
        if(ATTEMPT LESS ${MAX_ATTEMPTS})
            message(STATUS "File still locked, waiting before retry...")
            execute_process(COMMAND ${CMAKE_COMMAND} -E sleep 0.5)
        endif()

        math(EXPR ATTEMPT "${ATTEMPT} + 1")
    endwhile()

    # Final check
    if(EXISTS "${MEX_FILE}")
        message(WARNING "Could not delete MEX file after ${MAX_ATTEMPTS} attempts. File may still be in use.")
        message(WARNING "Please close MATLAB and try again, or delete manually: ${MEX_FILE}")
    endif()
else()
    message(STATUS "No old MEX file found to delete")
endif()

