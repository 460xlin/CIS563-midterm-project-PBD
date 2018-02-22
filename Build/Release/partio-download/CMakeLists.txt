cmake_minimum_required(VERSION 2.8.2)

project(partio-download NONE)

include(ExternalProject)
ExternalProject_Add(partio-download
                    GIT_REPOSITORY;https://github.com/wdas/partio.git;GIT_TAG;v1.1.0;UPDATE_DISCONNECTED;1;PATCH_COMMAND;/usr/bin/git;apply;/home/xinyulin/CIS563/cispba/Deps/partio.patch
                    SOURCE_DIR          "/home/xinyulin/CIS563/cispba/Build/Release/partio-src"
                    BINARY_DIR          "/home/xinyulin/CIS563/cispba/Build/Release/partio-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
