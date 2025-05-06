
CPMAddPackage(
        NAME pybind11
        GITHUB_REPOSITORY pybind/pybind11
        VERSION 2.10.4
)

if(pybind11_ADDED)
    message(STATUS "🎉 pybind11 v${pybind11_VERSION} added via CPM")
endif()
