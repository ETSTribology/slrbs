#
# Copyright 2021 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#

# Configure CPM cache location
set(CPM_SOURCE_CACHE_DEFAULT "${CMAKE_SOURCE_DIR}/build/_cpm_cache"
    CACHE STRING "CPM cache location")

if(NOT CPM_SOURCE_CACHE)
    set(CPM_SOURCE_CACHE ${CPM_SOURCE_CACHE_DEFAULT})
endif()

# Make cache folder if it doesn't exist
if(NOT EXISTS ${CPM_SOURCE_CACHE})
    file(MAKE_DIRECTORY ${CPM_SOURCE_CACHE})
endif()

message(STATUS "Using CPM cache directory: ${CPM_SOURCE_CACHE}")