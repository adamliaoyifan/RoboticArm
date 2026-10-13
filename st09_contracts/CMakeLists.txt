cmake_minimum_required(VERSION 3.8)
project(st09_contracts)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
# Same PYTHONPATH environment hook ament_python_install_package() would
# register, so sourcing the overlay puts the modules on sys.path.
_ament_cmake_python_register_environment_hook()

# The frozen C0 layout keeps the python package directly at the package
# root (repo-root ``import st09_contracts`` is relied on by the parallel
# test trees), so ament_python_install_package() -- which requires the
# nested <pkg>/<pkg>/ layout -- cannot be used. Install the modules
# directly into the python install dir instead; the environment hook
# from ament_cmake_python puts it on PYTHONPATH.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py")

# Frozen manifests stay addressable from the install tree for tooling
# (verify/freeze scripts and evidence checkers resolve them by hash).
install(FILES
  resolved_planning_contract.json
  st09_acceptance_manifest.json
  v2_field_audit.json
  restore_disposition.json
  DESTINATION share/${PROJECT_NAME})

ament_package()
