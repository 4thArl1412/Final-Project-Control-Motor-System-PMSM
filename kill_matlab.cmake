# Script to close MATLAB processes to release MEX file locks
# This is called before building MEX files

if(WIN32)
    message(STATUS "========== MATLAB Shutdown Sequence ==========")
    message(STATUS "Closing Simulink model windows...")

    # Close any running Simulink models gracefully using matlabroot command
    # First, try sending Ctrl+C to MATLAB windows to gracefully close them
    execute_process(
        COMMAND tasklist /FI "IMAGENAME eq MATLAB.exe" /FO CSV /NH
        OUTPUT_VARIABLE MATLAB_RUNNING
        OUTPUT_QUIET
        ERROR_QUIET
    )

    message(STATUS "Force killing MATLAB and Simulink processes...")

    # Kill ALL MATLAB-related processes with force
    execute_process(COMMAND taskkill /IM MATLAB.exe /F /T OUTPUT_QUIET ERROR_QUIET)
    execute_process(COMMAND taskkill /IM simulink.exe /F OUTPUT_QUIET ERROR_QUIET)
    execute_process(COMMAND taskkill /IM matlabstarter.exe /F OUTPUT_QUIET ERROR_QUIET)

    # Wait for processes to fully terminate
    message(STATUS "Waiting for processes to fully terminate (3 seconds)...")
    execute_process(COMMAND ${CMAKE_COMMAND} -E sleep 3)

    # Additional aggressive kill attempts
    execute_process(COMMAND taskkill /IM MATLAB.exe /F OUTPUT_QUIET ERROR_QUIET)
    execute_process(COMMAND taskkill /IM simulink.exe /F OUTPUT_QUIET ERROR_QUIET)

    execute_process(COMMAND ${CMAKE_COMMAND} -E sleep 1)

    message(STATUS "MATLAB processes terminated successfully")
    message(STATUS "=============================================")
endif()

