# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
# For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
#
# libRSF is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libRSF is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)

# do not compile the main functions of the apps used for testing
add_compile_definitions(TESTMODE)

# convenience macro to add tests
macro(package_add_test TESTNAME)
    # create an executable in which the tests will be stored
    add_executable(${TESTNAME} ${ARGN})
    # link the google test main function and the libRSF
    target_link_libraries(${TESTNAME} libRSF gtest_main)
    # add test to ctest, https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more info
    gtest_discover_tests(${TESTNAME}
        # set a working directory so that tests can find dataset files
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
endmacro()

package_add_test(Test_ICRA19_GNSS Test_ICRA19_GNSS.cpp ../applications/ICRA19_GNSS.cpp)

package_add_test(Test_Examples Test_Examples.cpp)