#!/usr/bin/env bash

# -u: error on unset variables; -o pipefail: propagate pipeline failures.
# Note: -e (exit on error) is intentionally omitted so that a failing sudo -n
# does not abort the script before we can print the warning message.
set -uo pipefail

OS=$(uname)

if [[ "$OS" == 'Linux' ]]; then
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd -P)"
    if [[ "$EUID" -eq 0 ]]; then
        # Already root — run directly, no sudo needed
        echo "Running as root, installing udev rules directly"
        "$SCRIPT_DIR/install_udev_rules.sh"
    else
        # Non-root — attempt passwordless sudo
        err=$(sudo -n "$SCRIPT_DIR/install_udev_rules.sh" 2>&1)
        EXIT_CODE=$?
        if [[ $EXIT_CODE -eq 0 ]]; then
            echo "Installed udev rules via passwordless sudo"
        else
            # sudo -n returns non-zero both when a password is required and when
            # install_udev_rules.sh itself fails, so we can't distinguish them.
            echo "WARNING: Could not install udev rules (sudo requires a password or install_udev_rules.sh failed)."
            [[ -n "$err" ]] && echo "Error output: $err"
            echo "To install manually, run: sudo $SCRIPT_DIR/install_udev_rules.sh"
        fi
    fi
fi
