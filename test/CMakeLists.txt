# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
# Copyright (C) 2026 Tim Pfeifer
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

# compile TestUtils once and share it across all test targets
add_library(libRSF_testutils OBJECT TestUtils.cpp)
target_link_libraries(libRSF_testutils PUBLIC libRSF gtest)

# convenience macro to add tests
macro(package_add_test TESTNAME)
    # create an executable in which the tests will be stored
    add_executable(${TESTNAME} ${ARGN})
    # suppress main() in application sources compiled as part of tests
    target_compile_definitions(${TESTNAME} PRIVATE TESTMODE)
    # link the google test main function and the shared test utilities
    target_link_libraries(${TESTNAME} PRIVATE gtest_main libRSF_testutils)
    # add test to ctest, https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more info
    gtest_discover_tests(${TESTNAME}
        DISCOVERY_MODE PRE_TEST
        # set a working directory so that tests can find dataset files
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
endmacro()

package_add_test(Test_Example_FG_Generic Test_Example_FG_Generic.cpp ../examples/Example_FG_Generic.cpp)

package_add_test(Test_Example_FG_Pseudorange Test_Example_FG_Pseudorange.cpp ../examples/Example_FG_Pseudorange.cpp)

package_add_test(Test_Example_FG_Range Test_Example_FG_Range.cpp ../examples/Example_FG_Range.cpp)

package_add_test(Test_Example_Marginalization Test_Example_Marginalization.cpp ../examples/Example_Marginalization.cpp)

package_add_test(Test_ICRA19_GNSS Test_ICRA19_GNSS.cpp ../applications/ICRA19_GNSS.cpp)

package_add_test(Test_IV19_GNSS Test_IV19_GNSS.cpp ../applications/IV19_GNSS.cpp)

package_add_test(Test_App_Robust_Models_1D Test_App_Robust_Models_1D.cpp ../applications/App_Robust_Models_1D.cpp)

package_add_test(Test_App_Robust_Models_2D Test_App_Robust_Models_2D.cpp ../applications/App_Robust_Models_2D.cpp)

package_add_test(Test_GMM_Estimation_1D Test_GMM_Estimation_1D.cpp)

package_add_test(Test_GMM_Estimation_2D Test_GMM_Estimation_2D.cpp)

package_add_test(Test_App_GNSS Test_App_GNSS.cpp ../applications/App_GNSS.cpp)
target_link_libraries(Test_App_GNSS PRIVATE libRSF_apppool)

package_add_test(Test_App_SLAM Test_App_SLAM.cpp ../applications/App_SLAM.cpp)
target_link_libraries(Test_App_SLAM PRIVATE libRSF_apppool)

package_add_test(Test_App_Ranging_2D Test_App_Ranging_2D.cpp ../applications/App_Ranging_2D.cpp)
target_link_libraries(Test_App_Ranging_2D PRIVATE libRSF_apppool)