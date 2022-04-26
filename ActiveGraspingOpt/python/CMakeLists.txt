cmake_minimum_required(VERSION 3.0.2)

find_package(PythonLibs 3.6 REQUIRED)

execute_process ( COMMAND 
         python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
         OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

INSTALL(
    FILES active_grasping_opt.py datalog.py active_grasping_params.py sigopt_executor.py
    DESTINATION "${PYTHON_SITE_PACKAGES}/active_grasping"
)
