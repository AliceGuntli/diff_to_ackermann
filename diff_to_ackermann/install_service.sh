#!/bin/bash

SERVICE_FILE="launch/diff_to_ack.service"
TARGET_DIR="/etc/systemd/system"

# Copy the service file
sudo cp $SERVICE_FILE $TARGET_DIR

# Reload systemd manager configuration
sudo systemctl daemon-reload

# Enable the service
sudo systemctl enable diff_to_ack.service

# Start the service
sudo systemctl start diff_to_ack.service

echo "Service diff_to_ack.service has been installed, enabled, and started."