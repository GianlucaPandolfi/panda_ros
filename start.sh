#! /bin/bash

echo "Choose controller"
echo "1) (joint space) inverse dynamics controller"
echo "2) Impedance controller"
echo "3) Inverse dynamics controller -> trajectory -> Impedance controller -> Deactivate inverse dyn"
echo "4) Only activate impedance controller"
echo "0) Exit"

read -r number

case "$number" in
  1)
    echo "Activating inverse dynamics controller"
    ros2 lifecycle set /inverse_dynamics_controller configure
    echo "Configured"
    sleep 1
    ros2 lifecycle set /inverse_dynamics_controller activate
    echo "Activated"
    echo ""
    ;;
  2)
    echo "Activating impedance controller"
    ros2 lifecycle set /impedance_controller configure
    echo "Configured"
    sleep 1
    ros2 lifecycle set /impedance_controller activate
    echo "Activated"
    echo ""
    ;;
  3)
    echo "Activating inverse dynamics controller"
    ros2 lifecycle set /inverse_dynamics_controller configure
    echo "Configured"
    sleep 1
    ros2 lifecycle set /inverse_dynamics_controller activate
    echo "Activated"
    echo "Executing trajectory"
    ros2 run panda_utils joint_traj_example 3 1.57 -0.78 0 -2 0 1 0
    echo "Activating impedance controller"
    ros2 lifecycle set /impedance_controller configure
    echo "Configured"
    sleep 1
    ros2 lifecycle set /impedance_controller activate
    sleep 3
    echo "Activated"
    echo "Deactivating inverse dynamics controller"
    ros2 lifecycle set /inverse_dynamics_controller deactivate
    echo ""
    ;;
  4)
    echo "Activating impedance controller"
    echo "3.."
    sleep 1
    echo "2.."
    sleep 1
    echo "1.."
    sleep 1
    ros2 lifecycle set /impedance_controller activate
    echo "Activated"
    echo ""
    ;;
  0)
    echo "Exiting"
    exit 0
    ;;
  *)
    echo "Unknown number: $number"
    ;;
esac

# read -p "Activate CLIK? (y/n): " answer
# case "$answer" in
#   [Yy]* )
#     echo "Activating CLIK";
#     ros2 lifecycle set /clik_cmd_pub configure;
#     echo "Configured";
#     sleep 1
#     ros2 lifecycle set /clik_cmd_pub activate;
#     echo "Activated";;
#   [Nn]* )
#     exit 1;;
#   * )
#     echo "Invalid input. Please answer y or n."; exit 1;;
# esac
