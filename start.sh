#! /bin/bash

echo "Activating inverse dynamics controller"
ros2 lifecycle set /inverse_dynamics_controller configure
echo "Configured"
sleep 1
ros2 lifecycle set /inverse_dynamics_controller activate
echo "Activated"
echo ""

# echo "Activating controller manager"
# ros2 lifecycle set /controller_manager configure
# sleep 1
# ros2 lifecycle set /controller_manager activate

read -p "Activate CLIK? (y/n): " answer
case "$answer" in
  [Yy]* )
    echo "Activating CLIK";
    ros2 lifecycle set /clik_cmd_pub configure;
    echo "Configured";
    sleep 1
    ros2 lifecycle set /clik_cmd_pub activate;
    echo "Activated";;
  [Nn]* )
    exit 1;;
  * )
    echo "Invalid input. Please answer y or n."; exit 1;;
esac
