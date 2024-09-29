#!/bin/bash

# RUN FIRST:
# cd scripts/
# touch setup_dependencies.sh
# chmod +x setup_dependencies.sh

INSTALL_REQ="install_req.txt"

if [ ! -f "$INSTALL_REQ" ]; then
    echo "install_req.txt not found!"
    exit 1
fi

while IFS= read -r line
do
    # Skip empty lines and comments
    [[ "$line" =~ ^#.*$ ]] && continue
    [[ -z "$line" ]] && continue

    # Parse the line
    method=$(echo "$line" | cut -d':' -f1)
    package=$(echo "$line" | cut -d':' -f2-)

    case "$method" in
        apt)
            echo "Installing apt package: $package"
            sudo apt update
            sudo apt install -y $package
            ;;
        pip)
            echo "Installing pip package: $package"
            pip3 install $package
            ;;
        snap)
            echo "Installing snap package: $package"
            sudo snap install $package
            ;;
        custom)
            echo "Executing custom installation: $package"
            eval $package
            ;;
        *)
            echo "Unknown installation method: $method"
            ;;
    esac
done < "$INSTALL_REQ"
