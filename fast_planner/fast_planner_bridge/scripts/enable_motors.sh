#!/bin/bash

# Auto-enable Hector motors on startup
echo "[EnableMotors] Waiting for /enable_motors service..."

# Wait for service to be available
while ! rosservice list | grep -q "/enable_motors"; do
    sleep 0.5
done

echo "[EnableMotors] Service found, enabling motors..."

# Enable motors
rosservice call /enable_motors "enable: true"

if [ $? -eq 0 ]; then
    echo "[EnableMotors] ✓ Motors enabled successfully!"
else
    echo "[EnableMotors] ✗ Failed to enable motors"
fi
