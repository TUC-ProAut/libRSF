# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
# For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
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
add_definitions(-DTESTMODE)
# After migration to CMAKE to 3.12.4, we can use the following instead
#add_compile_definitions(TESTMODE)

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

package_add_test(Test_Example_FG_Generic Test_Example_FG_Generic.cpp TestUtils.cpp ../examples/Example_FG_Generic.cpp)

package_add_test(Test_Example_FG_Pseudorange Test_Example_FG_Pseudorange.cpp TestUtils.cpp ../examples/Example_FG_Pseudorange.cpp)

package_add_test(Test_Example_FG_Range Test_Example_FG_Range.cpp TestUtils.cpp ../examples/Example_FG_Range.cpp)

package_add_test(Test_Example_Marginalization Test_Example_Marginalization.cpp TestUtils.cpp ../examples/Example_Marginalization.cpp)

package_add_test(Test_ICRA19_GNSS Test_ICRA19_GNSS.cpp TestUtils.cpp ../applications/ICRA19_GNSS.cpp)

package_add_test(Test_IV19_GNSS Test_IV19_GNSS.cpp TestUtils.cpp ../applications/IV19_GNSS.cpp)

package_add_test(Test_App_Robust_Models_1D Test_App_Robust_Models_1D.cpp TestUtils.cpp ../applications/App_Robust_Models_1D.cpp)

package_add_test(Test_App_Robust_Models_2D Test_App_Robust_Models_2D.cpp TestUtils.cpp ../applications/App_Robust_Models_2D.cpp)

package_add_test(Test_GMM_Estimation_1D Test_GMM_Estimation_1D.cpp TestUtils.cpp)

package_add_test(Test_GMM_Estimation_2D Test_GMM_Estimation_2D.cpp TestUtils.cpp)

package_add_test(Test_App_GNSS Test_App_GNSS.cpp TestUtils.cpp ../applications/App_GNSS.cpp ../applications/AppPool_Sensors.cpp ../applications/AppPool_Init.cpp ../applications/AppPool_Adaptive.cpp ../applications/AppPool_Defines.cpp ../applications/AppPool_Utility.cpp)

package_add_test(Test_App_SLAM Test_App_SLAM.cpp TestUtils.cpp ../applications/App_SLAM.cpp ../applications/AppPool_Sensors.cpp ../applications/AppPool_Init.cpp ../applications/AppPool_Adaptive.cpp ../applications/AppPool_Defines.cpp ../applications/AppPool_Utility.cpp)

package_add_test(Test_App_Ranging_2D Test_App_Ranging_2D.cpp TestUtils.cpp ../applications/App_Ranging_2D.cpp ../applications/AppPool_Sensors.cpp ../applications/AppPool_Init.cpp ../applications/AppPool_Adaptive.cpp ../applications/AppPool_Defines.cpp ../applications/AppPool_Utility.cpp)