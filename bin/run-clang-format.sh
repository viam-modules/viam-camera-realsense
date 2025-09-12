#!/bin/bash

# Copyright 2025 Viam Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Set up the linter
if command -v clang-format-19 &> /dev/null; then
    CLANG_FORMAT=clang-format-19
elif command -v clang-format &> /dev/null; then
    CLANG_FORMAT=clang-format
else
    # It's not yet installed, so let's get it!
    echo "Installing clang-format as a linter..."
    if [[ "$(uname)" == "Linux" ]]; then
        sudo apt update && sudo apt install -y clang-format-19
    elif [[ "$(uname)" == "Darwin" ]]; then
        brew install clang-format
    else
        echo "WARNING: installing the linter is not yet supported outside of Linux and Mac."
    fi

    # Re-check after installation
    if command -v clang-format-19 &> /dev/null; then
        CLANG_FORMAT=clang-format-19
    elif command -v clang-format &> /dev/null; then
        CLANG_FORMAT=clang-format
    else
        echo "ERROR: clang-format installation failed"
        exit 1
    fi
fi

# Function to ensure file ends with newline
ensure_final_newline() {
    local file="$1"
    # Check if file is non-empty and doesn't end with newline
    if [[ -s "$file" ]] && [[ $(tail -c1 "$file" | wc -l) -eq 0 ]]; then
        echo "" >> "$file"
        echo "Added final newline to: $file"
    fi
}

# Find and format files
find ./src -type f \( -name \*.cpp -o -name \*.hpp \) | while read -r file; do
    "$CLANG_FORMAT" -i --style=file "$@" "$file"
    ensure_final_newline "$file"
done

# Also check test files if they exist
if [[ -d "./test" ]]; then
    find ./test -type f \( -name \*.cpp -o -name \*.hpp \) | while read -r file; do
        "$CLANG_FORMAT" -i --style=file "$@" "$file"
        ensure_final_newline "$file"
    done
fi
